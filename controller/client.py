"""Test-Client für den RoboTwin-Controller: fährt den Roboter ohne Unity.

Kennt NUR das Protokoll (docs/PROTOCOL.md) – kein Code aus robotwin_controller.
So sieht jeder fremde Client aus, z. B. später die TcpConnection in Unity.

Start (der Controller muss laufen):
    python client.py               interaktiv, "help" zeigt die Befehle
    python client.py --demo        spielt eine feste Abfolge ab
"""

from __future__ import annotations

import argparse
import asyncio
import json
import sys
import threading
from typing import Optional

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 5005
TIMEOUT = 5.0   # Sekunden, die wir auf eine Antwort warten

HELP = """Befehle:
  move J1 J2 J3 J4 J5 J6 [speed]   alle Gelenke fahren (Grad; speed 0..1, Standard 1)
  home [speed]                     alle Gelenke auf 0°
  stop                             sofort anhalten
  override P                       Override in Prozent (1..100)
  estop                            NOT-HALT (sperrt alle Fahrbefehle)
  reset                            Not-Halt quittieren
  state                            aktuellen Zustand einmal anzeigen
  watch [Sekunden]                 Zustand mitlaufen lassen (Standard 3 s)
  raw JSON                         eine beliebige Zeile senden (zum Testen von Fehlern)
  help                             diese Hilfe
  quit                             beenden"""


class ControllerClient:
    """Verbindung zum Controller: senden, empfangen, Antworten den Befehlen zuordnen."""

    def __init__(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        self.reader = reader
        self.writer = writer
        self.next_id = 1
        self.role: Optional[str] = None
        self.joint_names: list[str] = []
        self.latest_state: Optional[dict] = None
        self.closed = asyncio.Event()
        self.state_received = asyncio.Event()

        # Offene Befehle: Id → Future, das mit der Antwort (ack/error) erfüllt wird.
        self._pending: dict[int, asyncio.Future] = {}
        self._watch_until = 0.0

    @classmethod
    async def connect(cls, host: str, port: int) -> "ControllerClient":
        reader, writer = await asyncio.open_connection(host, port)
        return cls(reader, writer)

    async def close(self) -> None:
        self.writer.close()
        try:
            await self.writer.wait_closed()
        except ConnectionError:
            pass

    # ---- Senden ----

    async def send_line(self, line: str) -> None:
        self.writer.write(line.encode("utf-8") + b"\n")
        await self.writer.drain()

    async def command(self, message_type: str, **fields) -> dict:
        """Sendet einen Befehl mit neuer Id und wartet auf die passende Antwort (ack oder error)."""
        command_id = self.next_id
        self.next_id += 1

        future = asyncio.get_running_loop().create_future()
        self._pending[command_id] = future
        message = {"type": message_type, "id": command_id, **fields}
        await self.send_line(json.dumps(message, separators=(",", ":")))
        try:
            return await asyncio.wait_for(future, TIMEOUT)
        finally:
            self._pending.pop(command_id, None)

    # ---- Empfangen ----

    async def receive_loop(self) -> None:
        """Liest alle Nachrichten des Controllers, bis die Verbindung endet."""
        try:
            while True:
                line = await self.reader.readline()
                if not line:
                    break
                self._handle(json.loads(line))
        except ConnectionError:
            pass
        finally:
            print("\nVerbindung zum Controller beendet.")
            self.closed.set()
            for future in self._pending.values():
                if not future.done():
                    future.set_exception(ConnectionError("Verbindung beendet"))

    def _handle(self, message: dict) -> None:
        message_type = message.get("type")

        if message_type == "hello":
            self.role = message["role"]
            self.joint_names = [joint["name"] for joint in message["joints"]]
            print(f"← hello: Protokoll {message['protocol']}, Rolle {self.role}, "
                  f"{len(self.joint_names)} Gelenke, {message['state_rate']} Hz")

        elif message_type == "role":
            self.role = message["role"]
            print(f"← Rolle geändert: {self.role}")

        elif message_type in ("ack", "error"):
            text = f"← {message_type} #{message['id']}"
            if message_type == "error":
                text += f"  {message['code']}: {message['message']}"
            print(text)
            future = self._pending.get(message.get("id"))
            if future is not None and not future.done():
                future.set_result(message)

        elif message_type == "state":
            previous = self.latest_state
            self.latest_state = message
            self.state_received.set()
            if previous is None or previous["status"] != message["status"]:
                reason = f" – {message['error']}" if message.get("error") else ""
                print(f"  Status: {message['status']}{reason}")
            loop = asyncio.get_running_loop()
            if loop.time() < self._watch_until and message["seq"] % 10 == 0:   # 5-mal pro Sekunde
                print("  " + format_state(message))

    # ---- Hilfen ----

    def watch(self, seconds: float) -> None:
        self._watch_until = asyncio.get_running_loop().time() + seconds

    async def wait_until_idle(self, timeout: float = 30.0) -> dict:
        """Wartet auf den ersten Zustand NACH jetzt, der 'idle' meldet."""
        async def wait() -> dict:
            start_seq = self.latest_state["seq"] if self.latest_state else -1
            while True:
                self.state_received.clear()
                await self.state_received.wait()
                state = self.latest_state
                if state["seq"] > start_seq and state["status"] == "idle":
                    return state
        return await asyncio.wait_for(wait(), timeout)


def format_state(state: dict) -> str:
    joints = "  ".join(f"J{i + 1} {angle:7.1f}" for i, angle in enumerate(state["joints"]))
    return f"[{state['status']:6}] {joints}   (seq {state['seq']}, t {state['timestamp']:.2f} s, Override {state['override']:.0f} %)"


def parse_number(text: str) -> float:
    # Komma als Dezimaltrennzeichen erlauben ("90,5") – für Menschen. Im Protokoll steht IMMER ein Punkt.
    return float(text.replace(",", "."))


# ---- Interaktiver Modus ----

async def handle_input(client: ControllerClient, line: str) -> bool:
    """Verarbeitet eine eingegebene Zeile. Gibt False zurück, wenn beendet werden soll."""
    parts = line.split()
    if not parts:
        return True
    name, args = parts[0].lower(), parts[1:]

    try:
        if name in ("quit", "exit"):
            return False
        if name == "help":
            print(HELP)
        elif name == "move":
            if len(args) not in (6, 7):
                print("move braucht 6 Winkel und optional speed, z. B.: move 0 -90 90 -90 -90 0 0.5")
                return True
            numbers = [parse_number(a) for a in args]
            fields = {"joints": numbers[:6]}
            if len(numbers) == 7:
                fields["speed"] = numbers[6]
            await client.command("move_joints", **fields)
        elif name == "home":
            fields = {"joints": [0.0] * 6}
            if args:
                fields["speed"] = parse_number(args[0])
            await client.command("move_joints", **fields)
        elif name == "stop":
            await client.command("stop")
        elif name == "estop":
            await client.command("emergency_stop")
        elif name == "reset":
            await client.command("reset")
        elif name == "override":
            if len(args) != 1:
                print("override braucht einen Wert, z. B.: override 50")
                return True
            await client.command("set_override", percent=parse_number(args[0]))
        elif name == "state":
            print("  " + format_state(client.latest_state) if client.latest_state else "  noch kein Zustand empfangen")
        elif name == "watch":
            client.watch(parse_number(args[0]) if args else 3.0)
        elif name == "raw":
            await client.send_line(line.split(None, 1)[1] if len(parts) > 1 else "")
        else:
            print(f"Unbekannter Befehl '{name}' – 'help' zeigt alle Befehle.")
    except ValueError:
        print("Ungültige Zahl.")
    except asyncio.TimeoutError:
        print("Keine Antwort vom Controller.")
    return True


async def interactive(client: ControllerClient) -> None:
    """Liest Eingaben, während im Hintergrund weiter Nachrichten empfangen werden.

    input() blockiert – in der Event-Loop würde es alles anhalten. Deshalb läuft es in einem
    eigenen Thread, der jede Zeile über call_soon_threadsafe an die Loop übergibt.
    Genau dieses Muster ("Thread übergibt an den Haupt-Thread") braucht Unity in M6.
    """
    loop = asyncio.get_running_loop()
    lines: asyncio.Queue[Optional[str]] = asyncio.Queue()

    def read_input() -> None:
        while True:
            try:
                line: Optional[str] = input()
            except EOFError:
                line = None   # Eingabe beendet (Strg+Z unter Windows, Strg+D unter Linux)
            loop.call_soon_threadsafe(lines.put_nowait, line)
            if line is None:
                return

    # daemon=True: Der Thread hält das Programm beim Beenden nicht fest.
    threading.Thread(target=read_input, daemon=True).start()
    print(HELP)

    while not client.closed.is_set():
        line_task = asyncio.ensure_future(lines.get())
        closed_task = asyncio.ensure_future(client.closed.wait())
        done, _ = await asyncio.wait({line_task, closed_task}, return_when=asyncio.FIRST_COMPLETED)
        closed_task.cancel()
        if line_task not in done:
            line_task.cancel()
            break
        line = line_task.result()
        if line is None or not await handle_input(client, line):
            break


# ---- Demo-Modus ----

async def demo(client: ControllerClient, stop_after: float = 1.0) -> bool:
    """Spielt eine feste Abfolge ab. Gibt True zurück, wenn alles wie erwartet verlief."""
    await client.state_received.wait()

    async def move(joints: list[float], label: str) -> None:
        print(f"\n▶ {label}: {joints}")
        reply = await client.command("move_joints", joints=joints)
        if reply["type"] != "ack":
            raise RuntimeError(f"unerwartet abgelehnt: {reply}")
        state = await client.wait_until_idle()
        print("  " + format_state(state))

    try:
        await client.command("set_override", percent=100)
        await move([0, -90, 90, -90, -90, 0], "Bereitschaftsstellung")
        await move([90, -45, 60, -30, 90, 45], "Testpose")

        print("\n▶ Ungültiges Ziel (Ellbogen 200°) – muss abgelehnt werden")
        reply = await client.command("move_joints", joints=[0, 0, 200, 0, 0, 0])
        if reply.get("code") != "joint_limit":
            raise RuntimeError(f"erwartet joint_limit, erhalten {reply}")

        print(f"\n▶ Override 30 %, nach Home fahren und nach {stop_after} s anhalten")
        await client.command("set_override", percent=30)
        await client.command("move_joints", joints=[0, 0, 0, 0, 0, 0])
        await asyncio.sleep(stop_after)
        await client.command("stop")
        state = await client.wait_until_idle()
        print("  angehalten bei: " + format_state(state))

        print("\n▶ Not-Halt: danach muss jeder Fahrbefehl abgelehnt werden, bis quittiert wird")
        await client.command("emergency_stop")
        reply = await client.command("move_joints", joints=[0, 0, 0, 0, 0, 0])
        if reply.get("code") != "emergency_stop_active":
            raise RuntimeError(f"erwartet emergency_stop_active, erhalten {reply}")
        await client.command("reset")

        await client.command("set_override", percent=100)
        await move([0, 0, 0, 0, 0, 0], "Home")
    except (RuntimeError, asyncio.TimeoutError, ConnectionError) as error:
        print(f"\nDemo fehlgeschlagen: {error}")
        return False

    print("\nDemo erfolgreich beendet.")
    return True


# ---- Start ----

async def run(host: str, port: int, run_demo: bool) -> int:
    try:
        client = await ControllerClient.connect(host, port)
    except OSError:
        print(f"Kein Controller unter {host}:{port} erreichbar. Läuft 'python -m robotwin_controller'?")
        return 1

    receiver = asyncio.create_task(client.receive_loop())
    try:
        if run_demo:
            return 0 if await demo(client) else 1
        await interactive(client)
        return 0
    finally:
        await client.close()
        receiver.cancel()


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Test-Client für den RoboTwin-Controller.")
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--demo", action="store_true", help="feste Abfolge abspielen statt interaktiv")
    args = parser.parse_args(argv)
    try:
        return asyncio.run(run(args.host, args.port, args.demo))
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
