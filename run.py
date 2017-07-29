#! /usr/bin/env python3
import krpc

conn = krpc.connect(name="spacelib")
vessel = conn.space_center.active_vessel
print(vessel.name)
