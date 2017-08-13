import logging
import math
import unittest

from spacelib import core, dynamics

core.init_logging()
logger = logging.getLogger(__name__)

class TestOrbits(unittest.TestCase):
    def setUp(self):
        # This is a live test which requires a kRPC connection and a vessel
        # in orbit
        self.conn = core.setup()
        self.v = self.conn.space_center.active_vessel
        self.assertIsNotNone(self.v)
        self.assertEqual(self.v.situation.name, "orbiting")

    def tearDown(self):
        self.conn.close()

    def test_construct(self):
        v = self.v
        ref_orbit = dynamics.Orbit.from_ksp_orbit(v.orbit)
        frame = v.orbit.body.non_rotating_reference_frame
        #frame = v.orbit.body.reference_frame
        constructed_orbit = dynamics.Orbit.from_pos_vel(v.orbit.body,
                v.position(frame), v.velocity(frame))

        # we're going for 1% deviation here, let's see how that goes
        d = lambda x: abs(x * 0.01)
        self.assertAlmostEqual(ref_orbit.a, constructed_orbit.a, delta=d(ref_orbit.a))
        self.assertAlmostEqual(ref_orbit.e, constructed_orbit.e, delta=d(ref_orbit.e))
        self.assertAlmostEqual(ref_orbit.nu, constructed_orbit.nu, delta=d(ref_orbit.nu))
        flight = v.flight(frame)
        logger.debug("reference lat: %s", flight.latitude)
        logger.debug("reference long: %s", flight.longitude)
        # bizarrely I'm sure the reference heading is wrong o_O
        # the heading I calculate agrees with the heading on my navball
        logger.debug("reference heading: %s", flight.heading)
        self.assertAlmostEqual(ref_orbit.i, constructed_orbit.i, delta=d(ref_orbit.i))
        self.assertAlmostEqual(ref_orbit.w, constructed_orbit.w, delta=d(ref_orbit.w))
        self.assertAlmostEqual(ref_orbit.n, constructed_orbit.n, delta=d(ref_orbit.n))

    def test_nu_at_t(self):
        period = self.v.orbit.period
        o = dynamics.Orbit.from_ksp_orbit(self.v.orbit)
        for i in range(0, 100, 17):
            t = self.conn.space_center.ut + period * (i/100)
            self.assertAlmostEqual(o.nu_at_t(t), self.v.orbit.true_anomaly_at_ut(t), delta=0.02 * math.pi)
