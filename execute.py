#! /usr/bin/env python3
import argparse

from spacelib.core import setup, init_logging
from spacelib import missions

init_logging()

parser = argparse.ArgumentParser()
parser.add_argument("plan", help="JSON plan file", nargs="?", default=None)
args = parser.parse_args()

conn = setup()

if args.plan:
    mission = missions.load_mission_from_file(args.plan)
    mission.execute(interactive=True)
else:
    missions.mission_repl()
