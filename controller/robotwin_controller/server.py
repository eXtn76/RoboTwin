"""asyncio-TCP-Server: nimmt Clients an, führt ihre Befehle aus und sendet zyklisch den Zustand.

Alles läuft in EINEM Thread. Zwei Arten von Aufgaben teilen ihn sich:
* pro Client eine Coroutine (_handle_client), die Zeilen liest und Befehle ausführt,
* eine Takt-Coroutine (_tick_loop), die den Roboter simuliert und "state" an alle sendet.
Gewechselt wird nur an "await"-Stellen. Code zwischen zwei awaits läuft daher ununterbrochen –
deshalb braucht der gemeinsame Roboterzustand keine Locks.
"""

from __future__ import annotations

import asyncio
import contextlib
import logging
import threading
from typing import Optional

from . import protocol
from .protocol import EmergencyStop, MoveJoints, ProtocolError, Reset, SetOverride, Stop
from .robot import CommandError, Robot

log = logging.getLogger("robotwin")

DEFAULT_HOST = "127.0.0.1"   # nur lokal erreichbar – das Protokoll hat keine Anmeldung
DEFAULT_PORT = 5005
DEFAULT_STATE_RATE = 50      # Hz

OWNER = "owner"
OBSERVER = "observer"

MAX_STEP = 0.1                       # nie mehr als 100 ms auf einmal simulieren (z. B. nach einem Hänger)
MAX_WRITE_BUFFER = 1024 * 1024       # liest ein Client nicht mehr mit, wird er nach 1 MiB Rückstau getrennt


class ClientConnection:
    """Ein verbundener Client: Lese- und Schreibende der TCP-Verbindung."""

    def __init__(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        self.reader = reader
        self.writer = writer
        peer = writer.get_extra_info("peername")
        self.name = f"{peer[0]}:{peer[1]}" if peer else "?"

    def send(self, message: dict) -> None:
        self.send_bytes(protocol.encode(message))

    def send_bytes(self, data: bytes) -> None:
        """Schreibt in den Sendepuffer, OHNE zu warten (kein await).

        Würden wir auf jeden Client warten (await drain), könnte ein einziger langsamer Client
        den Takt für alle aufhalten. Stattdessen: Puffer beobachten und hängende Clients trennen.
        """
        if self.writer.is_closing():
            return
        self.writer.write(data)
        if self.writer.transport.get_write_buffer_size() > MAX_WRITE_BUFFER:
            log.warning("%s liest nicht mehr mit – Verbindung wird getrennt", self.name)
            self.writer.close()


class RobotServer:
    """Der Controller: Roboter + TCP-Server + Takt."""

    def __init__(self, robot: Robot, host: str = DEFAULT_HOST, port: int = DEFAULT_PORT,
                 state_rate: int = DEFAULT_STATE_RATE) -> None:
        self.robot = robot
        self.host = host
        self.requested_port = port
        self.state_rate = state_rate

        # Reihenfolge = Verbindungsreihenfolge. Der Erste ist Besitzer (PROTOCOL.md, Abschnitt 4).
        self.clients: list[ClientConnection] = []

        self._server: Optional[asyncio.base_events.Server] = None
        self._tick_task: Optional[asyncio.Task] = None
        self._seq = 0

    @property
    def owner(self) -> Optional[ClientConnection]:
        return self.clients[0] if self.clients else None

    @property
    def port(self) -> int:
        """Tatsächlicher Port (bei port=0 wählt das Betriebssystem einen freien – praktisch für Tests)."""
        return self._server.sockets[0].getsockname()[1]

    # ---- Starten und Beenden ----

    async def start(self) -> None:
        # limit: maximale Zeilenlänge – schützt davor, dass ein Client endlose Zeilen schickt.
        self._server = await asyncio.start_server(
            self._handle_client, self.host, self.requested_port, limit=protocol.MAX_LINE_BYTES)
        self._tick_task = asyncio.create_task(self._tick_loop())
        log.info("Controller läuft auf %s:%d mit %d Hz", self.host, self.port, self.state_rate)

    async def run(self, console: bool = False) -> None:
        """Startet und läuft, bis abgebrochen wird (Strg+C) oder der Takt mit einem Fehler endet.

        console=True: Befehle im Konsolenfenster des Controllers annehmen (Not-Halt am Controller).
        """
        await self.start()
        if console:
            self._start_console()
        try:
            # gather: Endet eine der beiden Aufgaben mit einem Fehler, endet run() mit diesem Fehler.
            await asyncio.gather(self._server.serve_forever(), self._tick_task)
        finally:
            await self.stop()

    async def stop(self) -> None:
        if self._tick_task is not None:
            self._tick_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._tick_task
            self._tick_task = None
        if self._server is not None:
            self._server.close()
            for client in list(self.clients):
                client.writer.close()
            with contextlib.suppress(asyncio.TimeoutError):
                await asyncio.wait_for(self._server.wait_closed(), timeout=2.0)
            self._server = None

    # ---- Clients ----

    async def _handle_client(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        """Läuft einmal pro verbundenem Client, so lange die Verbindung besteht."""
        client = ClientConnection(reader, writer)
        self.clients.append(client)
        role = OWNER if client is self.owner else OBSERVER
        log.info("%s verbunden als %s", client.name, role)
        client.send(protocol.hello(role, self.robot.limits, self.state_rate))

        try:
            while True:
                try:
                    line = await reader.readline()   # wartet – derweil laufen Takt und andere Clients
                except ValueError:
                    # Zeile länger als MAX_LINE_BYTES: Fehler melden und Verbindung beenden.
                    client.send(protocol.error(None, "invalid_json", "Zeile zu lang."))
                    break
                if not line:
                    break                            # leere Rückgabe = Client hat die Verbindung beendet
                if line.strip():
                    self._handle_line(client, line)
        except ConnectionError:
            pass                                     # Verbindung abgerissen (z. B. Unity beendet)
        finally:
            self._remove_client(client)
            writer.close()
            with contextlib.suppress(ConnectionError):
                await writer.wait_closed()
            log.info("%s getrennt", client.name)

    def _handle_line(self, client: ClientConnection, line: bytes) -> None:
        """Eine empfangene Zeile verarbeiten. Kein await → läuft ununterbrochen (keine Race Conditions)."""
        try:
            command = protocol.parse_command(line)
        except ProtocolError as error:
            client.send(protocol.error(error.command_id, error.code, error.message))
            return

        # Rollen: Nur der Besitzer steuert – anhalten (auch per Not-Halt) darf jeder.
        if not isinstance(command, (Stop, EmergencyStop)) and client is not self.owner:
            client.send(protocol.error(command.id, "not_owner",
                                       "Nur der Besitzer darf steuern (stop und emergency_stop sind immer erlaubt)."))
            return

        try:
            if isinstance(command, MoveJoints):
                self.robot.move_joints(command.joints, command.speed)
            elif isinstance(command, Stop):
                self.robot.stop()
            elif isinstance(command, SetOverride):
                self.robot.set_override(command.percent)
            elif isinstance(command, EmergencyStop):
                self.robot.emergency_stop(f"Not-Halt von {client.name}")
                log.warning("NOT-HALT ausgelöst von %s", client.name)
            elif isinstance(command, Reset):
                if self.robot.emergency_stop_active:
                    log.info("Not-Halt quittiert von %s", client.name)
                self.robot.reset()
        except CommandError as error:
            client.send(protocol.error(command.id, error.code, error.message))
            return

        client.send(protocol.ack(command.id))

    def _remove_client(self, client: ClientConnection) -> None:
        was_owner = client is self.owner
        self.clients.remove(client)
        if not was_owner:
            return

        # Sicherheit: Bricht die Verbindung des Steuernden ab, hält der Roboter an.
        self.robot.stop()
        log.info("Besitzer %s getrennt – Roboter angehalten", client.name)
        if self.owner is not None:
            self.owner.send(protocol.role_changed(OWNER))
            log.info("%s ist neuer Besitzer", self.owner.name)

    # ---- Konsole des Controllers ("Not-Halt am Schaltschrank") ----

    def _start_console(self) -> None:
        """Liest Befehle aus dem Konsolenfenster.

        input() blockiert und läuft deshalb in einem eigenen Thread; jede Zeile wird per
        call_soon_threadsafe an die Event-Loop übergeben – dasselbe Muster wie im Test-Client.
        """
        loop = asyncio.get_running_loop()

        def read_console() -> None:
            while True:
                try:
                    line = input()
                except (EOFError, OSError):
                    return   # keine Konsole (z. B. Ausgabe umgeleitet) oder Eingabe beendet
                loop.call_soon_threadsafe(self.handle_console_command, line)

        threading.Thread(target=read_console, daemon=True).start()
        log.info("Konsole: 'estop' = Not-Halt, 'reset' = quittieren, 'status', 'help'")

    def handle_console_command(self, line: str) -> None:
        command = line.strip().lower()
        if command == "estop":
            self.robot.emergency_stop("Not-Halt am Controller")
            log.warning("NOT-HALT am Controller ausgelöst")
        elif command == "reset":
            if self.robot.emergency_stop_active:
                log.info("Not-Halt am Controller quittiert")
            self.robot.reset()
        elif command == "status":
            state = self.robot.state()
            joints = ", ".join(f"{angle:.1f}" for angle in state.joints)
            log.info("Status %s, Override %.0f %%, Gelenke [%s], Clients %d%s",
                     state.status.value, state.override, joints, len(self.clients),
                     f", Fehler: {state.error}" if state.error else "")
        elif command in ("help", "?"):
            log.info("Konsole: 'estop' = Not-Halt, 'reset' = quittieren, 'status', 'help'")
        elif command:
            log.info("Unbekannter Konsolenbefehl '%s' – 'help' zeigt alle.", command)

    # ---- Takt ----

    async def _tick_loop(self) -> None:
        """Simuliert den Roboter im festen Takt und sendet den Zustand an alle Clients."""
        loop = asyncio.get_running_loop()
        period = 1.0 / self.state_rate
        last = loop.time()
        next_tick = last

        while True:
            # Feste Zeitpunkte statt "sleep(period)": Kleine Verspätungen summieren sich so nicht auf.
            next_tick += period
            await asyncio.sleep(max(0.0, next_tick - loop.time()))

            now = loop.time()
            if now - next_tick > 5 * period:
                next_tick = now   # weit zurückgefallen (z. B. Rechner hing): nicht hinterherhetzen
            dt = min(now - last, MAX_STEP)
            last = now

            self.robot.step(dt)
            self._seq += 1
            self._broadcast(protocol.state(self._seq, self.robot.state()))

    def _broadcast(self, message: dict) -> None:
        data = protocol.encode(message)   # einmal kodieren, an alle senden
        for client in list(self.clients):
            client.send_bytes(data)
