"""Startet den Controller:   python -m robotwin_controller [--host H] [--port P] [--urdf DATEI] ..."""

from __future__ import annotations

import argparse
import asyncio
import logging
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

from .robot import DEFAULT_ACCELERATION, DEFAULT_URDF, Robot
from .server import DEFAULT_HOST, DEFAULT_PORT, DEFAULT_STATE_RATE, RobotServer

log = logging.getLogger("robotwin")


def parse_arguments(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="python -m robotwin_controller",
        description="Simulierter Robotercontroller für RoboTwin (Protokoll: docs/PROTOCOL.md).")
    parser.add_argument("--host", default=DEFAULT_HOST,
                        help=f"Adresse (Standard: {DEFAULT_HOST}; 0.0.0.0 = aus dem Netz erreichbar)")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"Port (Standard: {DEFAULT_PORT})")
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF, help="URDF-Datei mit den Achsgrenzen")
    parser.add_argument("--acceleration", type=float, default=DEFAULT_ACCELERATION,
                        help=f"Gelenkbeschleunigung in Grad/s² (Standard: {DEFAULT_ACCELERATION})")
    parser.add_argument("--rate", type=int, default=DEFAULT_STATE_RATE,
                        help=f"Takt der state-Nachrichten in Hz (Standard: {DEFAULT_STATE_RATE})")
    parser.add_argument("-v", "--verbose", action="store_true", help="ausführlichere Ausgaben")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_arguments(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)-7s %(message)s",
        datefmt="%H:%M:%S")

    try:
        robot = Robot.from_urdf(args.urdf, args.acceleration)
    except (OSError, ValueError, ET.ParseError) as error:
        log.error("URDF %s konnte nicht gelesen werden: %s", args.urdf, error)
        return 1
    log.info("Roboter aus %s: %s", args.urdf.name, ", ".join(j.name for j in robot.limits))

    server = RobotServer(robot, args.host, args.port, args.rate)
    try:
        asyncio.run(server.run(console=True))
    except KeyboardInterrupt:
        log.info("Beendet (Strg+C).")
    except OSError as error:
        # z. B. Port schon belegt (anderer Controller läuft noch)
        log.error("Server konnte nicht starten: %s", error)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
