import logging
import math

from .. import core
from ..util import cross_product, dot_product, vector_magnitude, vector_angle

logger = logging.getLogger(__name__)

class DynamicsError(Exception):
    pass

# Orbit class
# construct from a game orbit
# construct from a position + velocity vector (in x reference frame)
# get position+velocity vector at time

# Orbit notes (maths/our symbol = krpc name):
# a = semi_major_axis
# e = eccentricity
# i = inclination
# w (really omega) = argument_of_periapsis
# t = time of periapsis passage = ut + time_to_periapsis (% period?)
# n (really capital omega) = longitude_of_ascending_node
# u (really mu) = G * M
# P = period
# R_a, R_p = (radius at) apoapsis, periapsis
# nu = true anomaly
class Orbit:
    def __init__(self, body, a, e, i, w, nu, n, ref_t=None):
        self.body = body
        self.a = a
        self.e = e
        self.i = i
        self.w = w
        self.nu = nu
        self.n = n
        if ref_t is not None:
            self.ref_t = ref_t
        else:
            self.ref_t = core.conn.space_center.ut
        self.M_orig = None

    @classmethod
    def from_ksp_orbit(cls, o):
        return cls(
                o.body,
                o.semi_major_axis,
                o.eccentricity,
                o.inclination,
                o.argument_of_periapsis,
                o.true_anomaly,
                o.longitude_of_ascending_node)

    @classmethod
    def from_pos_vel(cls, body, pos, vel, frame=None):
        # need pos and vel in the reference frame used to map the orbital
        # inclination and so forth. going to assume that's
        # CelestialBody.non_rotating_reference_frame. if frame=None we assume
        # vectors are already in that frame.
        sc = core.conn.space_center
        if frame is not None:
            pos = sc.transform_position(pos, frame, body.non_rotating_reference_frame)
            vel = sc.transform_velocity(pos, vel, frame, body.non_rotating_reference_frame)
        # first of all we need the magnitudes of the pos and vel, and the angle between them
        # (zenith = y/gamma) (note: the flight path angle phi is the complement of zenith)
        r = vector_magnitude(pos)
        v = vector_magnitude(vel)
        y = vector_angle(pos, vel, degrees=False)

        u = sc.g * body.mass
        rvsqu = (r * v**2) / u
        a = 1 / ((2 / r) - (v**2 / u))
        e = math.sqrt(((rvsqu - 1)**2 * math.sin(y)**2) + math.cos(y)**2)
        nu = math.atan2(
                rvsqu * math.sin(y) * math.cos(y),
                rvsqu * math.sin(y)**2 - 1
                )

        # now we need the heading, latitude and longitude
        # note: Flight gives us these for a moving vessel, so we can use those values
        # to test (beware of difference between celestial and local longitude though)
        # lat/long are easy, just turn the vector into spherical polar coords
        # lat/long = elevation theta/azimuth phi, delta/lambda_2
        # reference frame: y goes through the north pole, x and z through the equator
        lat = math.asin(pos[1] / r)
        #logger.debug("lat: %s", math.degrees(lat))
        lon = math.atan2(pos[2], pos[0])
        #logger.debug("long: %s", math.degrees(lon))
        # heading we get from the position and velocity vectors
        # notated as beta
        # n is a vector in the north pole direction
        n = (0, 1, 0)
        pxv = cross_product(pos, vel)
        pxn = cross_product(pos, n)
        west = pxv[1] > 0
        head = math.acos(
                dot_product(pxv, pxn) / 
                (vector_magnitude(pxv) * vector_magnitude(pxn))
                )
        if west:
            head = math.pi * 2 - head
        #logger.debug("heading: %s", math.degrees(head))

        # from these we get the other orbital parameters
        i = math.acos(math.cos(lat) * math.sin(head))
        # l is the angular distance from ascending node to test point in orbit plane
        l = math.atan2(math.tan(lat), math.cos(head))
        w = l - nu # this assumes nu is -pi < nu < pi
        if w < 0:
            w += math.pi * 2
        dlon = math.atan2(math.tan(head), 1/math.sin(lat))
        n = lon - dlon
        flip_quad = math.pi/2 < head < 3*math.pi/2
        if (lat > 0 and flip_quad) or (lat <= 0 and not flip_quad):
            # correct quadrant for longitude measurement
            n += math.pi
        n %= math.pi * 2
        return cls(
                body,
                a,
                e,
                i,
                w,
                nu,
                n)

    @property
    def period(self):
        return math.sqrt(4 * math.pi**2 * self.a**3 / (core.conn.space_center.g * self.body.mass))

    def get_M_orig(self):
        """Get mean anomaly at reference time"""
        # via the eccentric anomaly
        top = math.sqrt(1 - self.e**2) * math.sin(self.nu)
        sin_E_orig = top / (1 + self.e * math.cos(self.nu))
        E_orig = math.atan2(top, self.e + math.cos(self.nu))
        self.M_orig = E_orig - self.e * sin_E_orig
        return self.M_orig

    def nu_at_t(self, t):
        """Get true anomaly at specified time"""
        # TODO handle hyperbolic orbits
        n = math.sqrt(core.conn.space_center.g * self.body.mass / self.a**3)

        if self.M_orig is None:
            self.get_M_orig()

        # mean anomaly at target
        M = (self.M_orig + n * (t - self.ref_t)) % (2 * math.pi)
        # now get the eccentric anomaly numerically, starting with mean anomaly
        # as the guess
        E = M
        for i in range(10):
            E = E - (E - self.e * math.sin(E) - M) / (1 - self.e * math.cos(E))
        # then true anomaly is
        nu = math.atan2(math.sqrt(1 - self.e**2) * math.sin(E), math.cos(E) - self.e)
        return nu


# Patched conic solver: find SOI changes and generate new Orbit
