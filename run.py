#! /usr/bin/env python3
import logging

from spacelib.core import setup
from spacelib import missions

logger = logging.getLogger()
logger.setLevel(logging.DEBUG)
handler = logging.StreamHandler()
handler.setLevel(logging.DEBUG)
logger.addHandler(handler)

conn = setup()

vessel = conn.space_center.active_vessel

missions.SuborbitalMission().execute(vessel)
