import logging
import math

import numpy as np

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
    def __init__(self, body, a, e, i, w, nu, n, ref_t=None,  M_orig=None):
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
        if M_orig is not None:
            self.M_orig = M_orig
        else:
            self.M_orig = self.get_M_orig()

    @classmethod
    def from_ksp_orbit(cls, o):
        return cls(
                o.body,
                o.semi_major_axis,
                o.eccentricity,
                o.inclination,
                o.argument_of_periapsis,
                o.true_anomaly,
                o.longitude_of_ascending_node,
                M_orig=o.mean_anomaly)

    @classmethod
    def from_pos_vel(cls, body, pos, vel, frame=None):
        # ref: https://downloads.rene-schwarz.com/download/M002-Cartesian_State_Vectors_to_Keplerian_Orbit_Elements.pdf
        # note: KSP coords: y is north, not z: y and z are switched
        # and coordinates are left-handed, hence use of -np.cross everywhere
        sc = core.conn.space_center
        if frame is not None:
            pos = sc.transform_position(pos, frame, body.non_rotating_reference_frame)
            vel = sc.transform_velocity(pos, vel, frame, body.non_rotating_reference_frame)
        pos = np.array(pos)
        vel = np.array(vel)

        mu = sc.g * body.mass
        h = -np.cross(pos, vel)
        pos_mag = np.linalg.norm(pos)
        # note this is the eccentricity vector!
        e = -np.cross(vel, h) / mu - pos / pos_mag
        e_mag = np.linalg.norm(e)

        n = -np.cross((0, 1, 0), h)
        nu = math.acos(np.dot(e, pos) / (e_mag * pos_mag))
        if np.dot(pos, vel) < 0:
            nu = -nu

        i = math.acos(h[1] / np.linalg.norm(h))
        E = 2 * math.atan2(math.tan(nu / 2), math.sqrt((1 + e_mag) / (1 - e_mag)))
        n_mag = np.linalg.norm(n)
        # capital omega
        asc = math.acos(n[0] / n_mag)
        if n[2] < 0:
            asc = 2 * math.pi - asc
        w = math.acos(np.dot(n, e) / (n_mag * e_mag))
        if e[1] < 0:
            w = 2 * math.pi - w
        M = E - e_mag * math.sin(E)
        vel_mag = np.linalg.norm(vel)
        a = 1 / ((2 / pos_mag) - (vel_mag**2 / mu))
        return cls(body,
                a,
                e_mag,
                i,
                w,
                nu,
                asc,
                M_orig=M)

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

    def anomalies_at_t(self, t):
        """Get eccentric and true anomaly at specified time"""
        # TODO handle hyperbolic orbits
        n = math.sqrt(core.conn.space_center.g * self.body.mass / self.a**3)

        # mean anomaly at target
        M = (self.M_orig + n * (t - self.ref_t)) % (2 * math.pi)
        # now get the eccentric anomaly numerically, starting with mean anomaly
        # as the guess
        E = M
        for i in range(10):
            E = E - (E - self.e * math.sin(E) - M) / (1 - self.e * math.cos(E))
        if E > math.pi:
            # correct to -pi to pi range
            E -= 2 * math.pi
        # then true anomaly is
        nu = math.atan2(math.sqrt(1 - self.e**2) * math.sin(E), math.cos(E) - self.e)
        return E, nu

    def vectors_at_t(self, t):
        """Get position and velocity vectors at a specified time."""
        # ref: https://downloads.rene-schwarz.com/download/M001-Keplerian_Orbit_Elements_to_Cartesian_State_Vectors.pdf
        sc = core.conn.space_center
        E, nu = self.anomalies_at_t(t)
        r_c = self.a * (1 - self.e * math.cos(E))
        # vectors in orbit frame (this isn't a frame kRPC natively knows about)
        o_pos = r_c * np.array((math.cos(nu), 0, math.sin(nu)))
        o_vel = (math.sqrt(sc.g * self.body.mass * self.a) / r_c) * np.array((
            -math.sin(E),
            0,
            math.sqrt(1 - self.e**2) * math.cos(E)))
        # transform to body.non_rotating_reference_frame with a manual rotation matrix
        cw = math.cos(self.w)
        cn = math.cos(self.n)
        ci = math.cos(self.i)
        sw = math.sin(self.w)
        sn = math.sin(self.n)
        si = math.sin(self.i)
        pos = (
                o_pos[0] * (cw*cn - sw*ci*sn) - o_pos[2] * (sw*cn + cw*ci*sn),
                o_pos[0] * (sw * si) + o_pos[2] * (cw * si),
                o_pos[0] * (cw*sn + sw*ci*cn) + o_pos[2] * (cw*ci*cn - sw*sn),
                )
        vel = (
                o_vel[0] * (cw*cn - sw*ci*sn) - o_vel[2] * (sw*cn + cw*ci*sn),
                o_vel[0] * (sw * si) + o_vel[2] * (cw * si),
                o_vel[0] * (cw*sn + sw*ci*cn) + o_vel[2] * (cw*ci*cn - sw*sn),
                )
        return pos, vel


# Patched conic solver: find SOI changes and generate new Orbit
