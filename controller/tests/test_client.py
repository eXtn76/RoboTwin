"""Ende-zu-Ende-Test: Der Test-Client (client.py) spielt seine Demo gegen einen echten Controller ab.

client.py kennt nur das Protokoll, nicht den Controller-Code. Läuft die Demo durch, passen
Client und Controller über das Protokoll zusammen – genau das muss später auch Unity leisten.

Ausführen im Ordner controller/:   python -m unittest
"""

import asyncio
import contextlib
import io
import unittest

import client
from robotwin_controller.robot import JointLimits, Robot
from robotwin_controller.server import RobotServer


class DemoTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        # Schneller Roboter (10-fache Geschwindigkeit), damit der Test nur kurz dauert.
        limits = [JointLimits(f"joint_{i + 1}", -360, 360, 1800, 1800) for i in range(6)]
        limits[2] = JointLimits("elbow_joint", -180, 180, 1800, 1800)
        self.server = RobotServer(Robot(limits), "127.0.0.1", port=0, state_rate=50)
        await self.server.start()

    async def asyncTearDown(self):
        await self.server.stop()

    async def test_demo_runs_successfully(self):
        controller = await client.ControllerClient.connect("127.0.0.1", self.server.port)
        receiver = asyncio.create_task(controller.receive_loop())
        try:
            # Ausgaben der Demo abfangen, damit die Testausgabe übersichtlich bleibt.
            with contextlib.redirect_stdout(io.StringIO()):
                success = await client.demo(controller, stop_after=0.05)
            self.assertTrue(success)
            self.assertEqual(controller.role, "owner")
            self.assertEqual(len(controller.joint_names), 6)
        finally:
            await controller.close()
            receiver.cancel()


if __name__ == "__main__":
    unittest.main()
