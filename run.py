#! /usr/bin/env python3
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

conn = setup()

vessel = conn.space_center.active_vessel

missions.OrbitalMission().execute(vessel)
