"""Tests für robotwin_controller.server – mit echten TCP-Verbindungen auf 127.0.0.1.

IsolatedAsyncioTestCase startet für jeden Test eine eigene Event-Loop, sodass
Test-Methoden "async def" sein und "await" benutzen dürfen.

Ausführen im Ordner controller/:   python -m unittest
"""

import asyncio
import json
import unittest

from robotwin_controller.robot import JointLimits, Robot
from robotwin_controller.server import RobotServer

TIMEOUT = 3.0   # Sekunden – großzügig, damit langsame Rechner nicht scheitern


def make_robot():
    limits = [JointLimits(f"joint_{i + 1}", -360, 360, 180, 180) for i in range(6)]
    limits[2] = JointLimits("elbow_joint", -180, 180, 180, 180)
    return Robot(limits)


class TestClient:
    """Minimaler Client für die Tests: Zeilen senden, Nachrichten eines bestimmten Typs lesen."""

    def __init__(self, reader, writer):
        self.reader = reader
        self.writer = writer

    @classmethod
    async def connect(cls, port):
        reader, writer = await asyncio.open_connection("127.0.0.1", port)
        return cls(reader, writer)

    async def send(self, text):
        self.writer.write(text.encode("utf-8") + b"\n")
        await self.writer.drain()

    async def receive(self, message_type, timeout=TIMEOUT):
        """Liest Nachrichten, bis eine vom gewünschten Typ kommt (state-Nachrichten dazwischen werden übersprungen)."""
        async def read():
            while True:
                line = await self.reader.readline()
                if not line:
                    raise ConnectionError("Verbindung geschlossen")
                message = json.loads(line)
                if message["type"] == message_type:
                    return message
        return await asyncio.wait_for(read(), timeout)

    async def wait_for_state(self, predicate, timeout=TIMEOUT):
        async def read():
            while True:
                message = await self.receive("state")
                if predicate(message):
                    return message
        return await asyncio.wait_for(read(), timeout)

    async def close(self):
        self.writer.close()
        try:
            await self.writer.wait_closed()
        except ConnectionError:
            pass


class ServerTestCase(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.robot = make_robot()
        self.server = RobotServer(self.robot, "127.0.0.1", port=0, state_rate=50)   # Port 0 = freien wählen
        await self.server.start()
        self.clients = []

    async def asyncTearDown(self):
        for client in self.clients:
            await client.close()
        await self.server.stop()

    async def connect(self):
        client = await TestClient.connect(self.server.port)
        self.clients.append(client)
        return client


class HelloAndRolesTests(ServerTestCase):
    async def test_first_client_gets_hello_as_owner(self):
        client = await self.connect()
        hello = await client.receive("hello")
        self.assertEqual(hello["protocol"], 1)
        self.assertEqual(hello["role"], "owner")
        self.assertEqual(hello["state_rate"], 50)
        self.assertEqual(len(hello["joints"]), 6)
        self.assertEqual(hello["joints"][2]["max"], 180)

    async def test_second_client_is_observer(self):
        await (await self.connect()).receive("hello")
        observer = await self.connect()
        self.assertEqual((await observer.receive("hello"))["role"], "observer")

    async def test_observer_may_not_move(self):
        owner = await self.connect()
        await owner.receive("hello")
        observer = await self.connect()
        await observer.receive("hello")

        await observer.send('{"type":"move_joints","id":5,"joints":[10,0,0,0,0,0]}')
        error = await observer.receive("error")
        self.assertEqual(error["code"], "not_owner")
        self.assertEqual(error["id"], 5)

    async def test_observer_may_stop(self):
        owner = await self.connect()
        await owner.receive("hello")
        observer = await self.connect()
        await observer.receive("hello")

        await owner.send('{"type":"move_joints","id":1,"joints":[300,0,0,0,0,0]}')
        await owner.receive("ack")
        await observer.send('{"type":"stop","id":1}')
        self.assertEqual((await observer.receive("ack"))["id"], 1)
        await observer.wait_for_state(lambda s: s["status"] == "idle")

    async def test_owner_disconnect_stops_robot_and_promotes_observer(self):
        owner = await self.connect()
        await owner.receive("hello")
        observer = await self.connect()
        await observer.receive("hello")

        await owner.send('{"type":"move_joints","id":1,"joints":[300,0,0,0,0,0]}')
        await owner.receive("ack")
        await observer.wait_for_state(lambda s: s["status"] == "moving")

        await owner.close()
        self.clients.remove(owner)

        self.assertEqual((await observer.receive("role"))["role"], "owner")
        await observer.wait_for_state(lambda s: s["status"] == "idle")
        await observer.send('{"type":"move_joints","id":2,"joints":[0,0,0,0,0,0]}')
        await observer.receive("ack")   # darf jetzt steuern


class EmergencyStopTests(ServerTestCase):
    async def asyncSetUp(self):
        await super().asyncSetUp()
        self.owner = await self.connect()
        await self.owner.receive("hello")
        self.observer = await self.connect()
        await self.observer.receive("hello")

    async def test_observer_may_trigger_emergency_stop(self):
        await self.owner.send('{"type":"move_joints","id":1,"joints":[300,0,0,0,0,0]}')
        await self.owner.receive("ack")
        await self.observer.send('{"type":"emergency_stop","id":1}')
        await self.observer.receive("ack")
        state = await self.owner.wait_for_state(lambda s: s["status"] == "error")
        self.assertIn("Not-Halt", state["error"])

    async def test_moves_rejected_until_owner_resets(self):
        await self.observer.send('{"type":"emergency_stop","id":1}')
        await self.observer.receive("ack")

        await self.owner.send('{"type":"move_joints","id":2,"joints":[10,0,0,0,0,0]}')
        self.assertEqual((await self.owner.receive("error"))["code"], "emergency_stop_active")

        await self.observer.send('{"type":"reset","id":2}')   # Beobachter darf nicht quittieren
        self.assertEqual((await self.observer.receive("error"))["code"], "not_owner")

        await self.owner.send('{"type":"reset","id":3}')
        await self.owner.receive("ack")
        await self.owner.wait_for_state(lambda s: s["status"] == "idle" and s["error"] is None)
        await self.owner.send('{"type":"move_joints","id":4,"joints":[10,0,0,0,0,0]}')
        await self.owner.receive("ack")

    async def test_console_emergency_stop_and_reset(self):
        self.server.handle_console_command("estop")
        state = await self.owner.wait_for_state(lambda s: s["status"] == "error")
        self.assertEqual(state["error"], "Not-Halt am Controller")
        self.server.handle_console_command("reset")
        await self.owner.wait_for_state(lambda s: s["status"] == "idle")


class StateTests(ServerTestCase):
    async def test_states_arrive_at_about_50_hz_with_increasing_seq(self):
        """Eine Sekunde lang ALLE Zustände mitlesen und über den ganzen Zeitraum mitteln.

        Nicht "1 s schlafen, dann einen lesen": Während des Schlafens stauen sich die Nachrichten
        im Empfangspuffer, und man läse den ältesten statt des neuesten. Und nicht über wenige Takte
        rechnen: Unter Windows haben Timer ~15,6 ms Auflösung, einzelne Abstände schwanken stark –
        nur der Mittelwert ist 50 Hz.
        """
        client = await self.connect()
        await client.receive("hello")

        loop = asyncio.get_running_loop()
        states = []
        end = loop.time() + 1.0
        while loop.time() < end:
            states.append(await client.receive("state"))

        sequence = [s["seq"] for s in states]
        self.assertEqual(sequence, list(range(sequence[0], sequence[0] + len(sequence))),
                         "seq muss lückenlos um 1 steigen")

        rate = (states[-1]["seq"] - states[0]["seq"]) / (states[-1]["timestamp"] - states[0]["timestamp"])
        self.assertAlmostEqual(rate, 50, delta=5)
        self.assertGreaterEqual(len(states), 40)   # in 1 s: ~50

    async def test_all_clients_receive_states(self):
        a = await self.connect()
        b = await self.connect()
        await a.receive("state")
        await b.receive("state")


class CommandTests(ServerTestCase):
    async def asyncSetUp(self):
        await super().asyncSetUp()
        self.client = await self.connect()
        await self.client.receive("hello")

    async def test_move_joints_is_acked_and_reaches_target(self):
        await self.client.send('{"type":"move_joints","id":1,"joints":[10,-10,10,0,0,0]}')
        self.assertEqual((await self.client.receive("ack"))["id"], 1)
        await self.client.wait_for_state(lambda s: s["status"] == "moving")
        final = await self.client.wait_for_state(lambda s: s["status"] == "idle")
        self.assertEqual(final["joints"], [10, -10, 10, 0, 0, 0])

    async def test_joint_limit_is_rejected(self):
        await self.client.send('{"type":"move_joints","id":2,"joints":[0,0,200,0,0,0]}')
        error = await self.client.receive("error")
        self.assertEqual(error["code"], "joint_limit")
        self.assertEqual(error["id"], 2)

    async def test_override_is_reported_in_state(self):
        await self.client.send('{"type":"set_override","id":3,"percent":25}')
        await self.client.receive("ack")
        await self.client.wait_for_state(lambda s: s["override"] == 25)

    async def test_invalid_json_gives_error_and_connection_stays_open(self):
        await self.client.send("das ist kein JSON")
        error = await self.client.receive("error")
        self.assertEqual(error["code"], "invalid_json")
        self.assertIsNone(error["id"])

        await self.client.send('{"type":"stop","id":4}')   # Verbindung funktioniert weiter
        self.assertEqual((await self.client.receive("ack"))["id"], 4)

    async def test_garbage_does_not_crash_the_server(self):
        for text in ("", "{", "[]", "null", '{"type":7,"id":1}', '{"type":"move_joints","id":1,"joints":null}'):
            await self.client.send(text)
        await self.client.send('{"type":"stop","id":99}')
        self.assertEqual((await self.client.receive("ack"))["id"], 99)

    async def test_too_long_line_closes_connection(self):
        self.client.writer.write(b"x" * (70 * 1024))   # länger als 64 KiB, ohne Zeilenende
        await self.client.writer.drain()
        error = await self.client.receive("error")
        self.assertEqual(error["code"], "invalid_json")
        with self.assertRaises(ConnectionError):
            await self.client.receive("state")


if __name__ == "__main__":
    unittest.main()
