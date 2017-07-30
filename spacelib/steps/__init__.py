import logging
import math
import time

from .. import core

logger = logging.getLogger(__name__)


class StepFailed(Exception):
    pass


class Step:
    """A single step in a mission."""
    def execute(self, v):
        """Run this step of a mission.

        Takes the vessel to run on."""
        raise NotImplementedError

    def __str__(self):
        return "Step"


class LaunchToApoapsis(Step):
    def __init__(self, height):
        # TODO check that target height > body upper atmosphere limit
        # TODO allow different angled orbits - force roll
        # TODO generalise for other planets
        # TODO check for control loss (large deviation of angle from target)
        # TODO time acceleration
        # TODO improve the pitchover
        # TODO autopilot that can actually fly straight
        self.height = height

    def __str__(self):
        return "Launch to {}".format(self.height)

    def execute(self, v):
        logger.info("Begun launch step")
        self.v = v
        # Set up needed telemetry
        c = core.conn
        self.throttle = 1
        self.q_limit = 1
        self.solid_fuel_stream = None
        self.liquid_fuel_stream = None
        self.started_gravity_turn = False
        self.last_altitude = 0
        self.descending_count = 0
        try:
            with c.stream(getattr, v.flight(), "mean_altitude") as altitude, \
                    c.stream(getattr, v.orbit, "apoapsis_altitude") as apoapsis, \
                    c.stream(getattr, v.flight(), "dynamic_pressure") as q:
                self.prep_launch()
                logger.info("Launch!")
                v.control.activate_next_stage()
                while True:
                    self.do_failure_check(altitude)
                    self.do_staging()
                    on_target = self.do_apoapsis_check(apoapsis)
                    self.do_gravity_turn(altitude)
                    self.do_q_check(q)
                    if on_target and self.do_atmosphere_check(altitude):
                        logger.info("Exited atmosphere and attaining target altitude. Exiting.")
                        v.auto_pilot.disengage()
                        return
                    v.control.throttle = self.throttle * self.q_limit
                    time.sleep(0.1)
        finally:
            self.close_streams()

    def do_failure_check(self, altitude):
        a = altitude()
        if a < self.last_altitude:
            logger.warning("Descent warning")
            self.descending_count += 1
        else:
            self.descending_count = 0
        self.last_altitude = a
        if self.descending_count > 10:
            raise StepFailed("Descent detected")

    def do_atmosphere_check(self, altitude):
        return altitude() >= 70000

    def do_apoapsis_check(self, apoapsis):
        res = False
        a = apoapsis()
        if a >= self.height * 0.999:
            self.throttle = 0
            res = True
        elif a > self.height * 0.99:
            # scale throttle
            self.throttle = min(1, 0.1 + (self.height - a) / (self.height * 0.01))
        else:
            self.throttle = 1
        return res

    def do_gravity_turn(self, altitude):
        a = altitude()
        if a < 10000:
            return
        if not self.started_gravity_turn:
            logger.info("Starting gravity turn")
            # Lazy pitchover
            # Set the vessel's reference frame to orbital rather than surface
            ap = self.v.auto_pilot
            ap.reference_frame = self.v.orbital_reference_frame
            ap.target_direction = (0, 1, 0)
            self.started_gravity_turn = True

    def do_staging(self):
        # Update streams
        # We want to find the resources we'll drop if we stage
        stage = self.v.control.current_stage - 1
        if self.solid_fuel_stream is None:
            res = self.v.resources_in_decouple_stage(stage, False)
            self.solid_fuel_stream = core.conn.add_stream(res.amount, "SolidFuel")
        if self.liquid_fuel_stream is None:
            res = self.v.resources_in_decouple_stage(stage, False)
            self.liquid_fuel_stream = core.conn.add_stream(res.amount, "LiquidFuel")
        # If the current stage has no liquid fuel AND no solid fuel, stage
        if self.solid_fuel_stream() < 0.1 and self.liquid_fuel_stream() < 0.1:
            logger.info("Stage empty, staging")
            self.v.control.throttle = 0
            self.close_streams()
            time.sleep(0.5)
            self.v.control.activate_next_stage()
            time.sleep(0.5)
            # Check if we have any fuel left
            r = self.v.resources
            if r.amount("SolidFuel") < 0.1 and r.amount("LiquidFuel") < 0.1:
                raise StepFailed("Out of fuel")

    def close_streams(self):
        if self.solid_fuel_stream:
            self.solid_fuel_stream.remove()
            self.solid_fuel_stream = None
        if self.liquid_fuel_stream:
            self.liquid_fuel_stream.remove()
            self.liquid_fuel_stream = None

    def do_q_check(self, q_stream):
        Q_TARGET = 20000
        q = q_stream()
        if q > Q_TARGET:
            self.q_limit = 0
        elif q > Q_TARGET * 0.9:
            if self.q_limit == 1:
                logger.info("Throttling for dynamic pressure limitation")
            self.q_limit = (Q_TARGET - q) / (Q_TARGET * 0.1)
        else:
            if self.q_limit != 1:
                logger.info("Ended dynamic pressure throttling")
            self.q_limit = 1

    def prep_launch(self):
        logger.info("Prepping for launch")
        self.v.auto_pilot.reference_frame = self.v.surface_reference_frame
        self.v.auto_pilot.target_pitch_and_heading(90, 90)
        self.v.auto_pilot.engage()
        self.v.control.throttle = 1
        time.sleep(1)


class OrbitalAdjustment(Step):
    def __init__(self, when, semi_major):
        self.when = when
        self.semi_major = semi_major

    def __str__(self):
        return "Adjust orbit to semi-major axis {} at {}".format(self.semi_major, self.when)

    def execute(self, v):
        logger.info("Adding orbit adjust node")
        # use vis-viva to get dv required to shift orbit with current semi-
        # major axis to orbit with target semi_major axis
        # semi-major axis is half the sum of apoapsis + periapsis
        mu = v.orbit.body.gravitational_parameter
        if self.when == "apoapsis":
            r = v.orbit.apoapsis
            t = v.orbit.time_to_apoapsis
        elif self.when == "periapsis":
            r = v.orbit.periapsis
            t = v.orbit.time_to_periapsis
        elif isinstance(self.when, int):
            t = self.when
            anomaly = v.orbit.true_anomaly_at_ut(core.conn.space_center.ut + t)
            r = v.orbit.radius_at_true_anomaly(anomaly)
        else:
            raise StepFailed("unknown target time")
        a1 = v.orbit.semi_major_axis
        a2 = self.semi_major
        v1 = math.sqrt(mu * ((2 / r) - (1 / a1)))
        v2 = math.sqrt(mu * ((2 / r) - (1 / a2)))
        dv = v2 - v1
        v.control.add_node(core.conn.space_center.ut + t,
                prograde=dv)


        
class Circularise(OrbitalAdjustment):
    def __init__(self, when):
        super().__init__(when, 0)

    def __str__(self):
        return "Circularise orbit at {}".format(self.when)

    def execute(self, v):
        logger.info("Adding circularisation node")
        if self.when == "apoapsis":
            self.semi_major = v.orbit.apoapsis
        elif self.when == "periapsis":
            self.semi_major = v.orbit.periapsis
        else:
            raise StepFailed("unknown target time")
        super().execute(v)


class ChangePeriapsis(OrbitalAdjustment):
    def __init__(self, when, periapsis):
        super().__init__(when, 0)
        self.periapsis = periapsis

    def __str__(self):
        return "Adjust periapsis to {} at {}".format(self.periapsis, self.when)

    def execute(self, v):
        logger.info("Adding change periapsis node")
        # treat the supplied value as relative to the surface
        self.periapsis += v.orbit.body.equatorial_radius
        if self.when == "apoapsis":
            self.semi_major = (v.orbit.apoapsis + self.periapsis) / 2
        elif isinstance(self.when, int):
            anomaly = v.orbit.true_anomaly_at_ut(core.conn.space_center.ut + self.when)
            r = v.orbit.radius_at_true_anomaly(anomaly)
            self.semi_major = (r + self.periapsis) / 2
        super().execute(v)


class ExecuteNode(Step):
    # TODO keep an eye on orientation and adjust if it gets too far off
    # TODO generalise the auto-stager from launch to work here too

    def __str__(self):
        return "Execute next node"

    def execute(self, v):
        logger.info("Orientating for node execution")
        if not v.control.nodes:
            raise StepFailed("no node to execute")
        node = v.control.nodes[0]
        ap = v.auto_pilot
        ap.reference_frame = node.reference_frame
        ap.target_direction = node.burn_vector(node.reference_frame)
        ap.engage()
        time.sleep(0.1)
        with core.conn.stream(getattr, ap, "error") as error:
            while error() > 5:
                time.sleep(1)
        ap.wait()

        logger.info("Warping to circularisation burn")
        # get burn time
        F = v.available_thrust
        isp = v.specific_impulse * 9.81
        m0 = v.mass
        dv = node.delta_v
        m1 = m0 / math.exp(dv / isp)
        flow_rate = F / isp
        burn_time = (m0 - m1) / flow_rate

        burn_ut = node.ut - burn_time / 2
        # 5 second lead-in
        core.conn.space_center.warp_to(burn_ut - 5)

        with core.conn.stream(getattr, core.conn.space_center, "ut") as ut:
            while ut() < (burn_ut - 1):
                time.sleep(1)
            while ut() < burn_ut:
                time.sleep(0.1)

        logger.info("Executing burn")
        with core.conn.stream(getattr, node, "remaining_delta_v") as remaining_burn, \
                core.conn.stream(node.remaining_burn_vector, node.reference_frame) as vector:
            v.control.throttle = 1
            while remaining_burn() > 0.5:
                ap.target_direction = vector()
                if remaining_burn() < 5:
                    v.control.throttle = min(1, 0.1 + (remaining_burn() / (dv * 0.01)))
            v.control.throttle = 0
            ap.disengage()
        node.remove()
        logger.info("Done")


class ControllerConfirm(Step):
    def __str__(self):
        return "Wait for controller confirmation"

    def execute(self, v):
        input("Press enter to continue mission.")
    
    
class DropStagesAndOpenParachute(Step):
    # TODO warp to atmosphere
    def __str__(self):
        return "Wait for descent and open parachutes"

    def execute(self, v):
        logger.info("Begun parachute step")
        # find our parachutes
        parachutes = v.parts.parachutes
        if not parachutes:
            raise StepFailed("No parachutes, prepare for bumpy landing")
        stage = parachutes[0].part.stage
        v.control.throttle = 0
        time.sleep(0.5)
        # stage to the stage before parachute activation
        while v.control.current_stage > (stage + 1):
            logger.info("Dropping stage")
            v.control.activate_next_stage()
        ap = v.auto_pilot
        ap.reference_frame = v.orbital_reference_frame
        ap.target_direction = (0, -1, 0)
        ap.engage()
        with core.conn.stream(getattr, v.flight(), "mean_altitude") as altitude:
            logger.info("Waiting for parachute altitude")
            while altitude() > 5000:
                time.sleep(1)
            while altitude() > 2500:
                time.sleep(0.1)
            logger.info("Opening parachutes")
            v.control.activate_next_stage()
            ap.disengage()


class LaunchPrep(Step):
    def execute(self, v):
        logger.info("Prepping for launch")
        v.auto_pilot.target_pitch_and_heading(90, 90)
        v.auto_pilot.engage()
        v.control.throttle = 0.7
        time.sleep(1)


class Launch(Step):
    def execute(self, v):
        logger.info("Lift-off!")
        v.control.activate_next_stage()


class GravityTurn(Step):
    def execute(self, v):
        logger.info("Waiting for 10,000m")
        last = 0
        while v.flight().mean_altitude < 10000:
            a = v.flight().mean_altitude
            if a < last:
                raise StepFailed("altitude decreasing unexpectedly")
            time.sleep(1)

        logger.info("Gravity turn")
        v.auto_pilot.target_pitch_and_heading(60, 90)


class WaitForEmpty(Step):
    def execute(self, v):
        logger.info("Waiting for fuel to empty")
        while v.resources.amount("LiquidFuel") > 0.1:
            time.sleep(1)

        logger.info("Staging")
        v.control.activate_next_stage()
        v.auto_pilot.disengage()


class WaitForParachute(Step):
    def execute(self, v):
        logger.info("Waiting for descent to activate parachute")
        with core.conn.stream(getattr, v.flight(), "surface_altitude") as a:
            while a() > 5000:
                time.sleep(1)
            while a() > 2500:
                time.sleep(0.1)

        logger.info("Activating parachute")
        v.control.activate_next_stage()


class WaitForLanding(Step):
    def execute(self, v):
        logger.info("Waiting for landing")
        while v.flight(v.orbit.body.reference_frame).vertical_speed < -0.1:
            time.sleep(1)
        logger.info("Landed!")


class AbortEngines(Step):
    def execute(self, v):
        logger.error("ABORTING")
        v.control.throttle = 0
        v.control.activate_next_stage()
