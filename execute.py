#! /usr/bin/env python3
import argparse
import logging

from spacelib.core import setup
from spacelib import missions

logger = logging.getLogger()
logger.setLevel(logging.DEBUG)
handler = logging.StreamHandler()
handler.setLevel(logging.DEBUG)
formatter = logging.Formatter("%(asctime)s|%(name)s|%(levelname)s\t%(message)s")
handler.setFormatter(formatter)
logger.addHandler(handler)

parser = argparse.ArgumentParser()
parser.add_argument("plan", help="JSON plan file")
args = parser.parse_args()

conn = setup()

mission = missions.load_mission(args.plan)
mission.execute(interactive=True)
