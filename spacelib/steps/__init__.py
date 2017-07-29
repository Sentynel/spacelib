import logging
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


class LaunchToApoapsis(Step):
    def __init__(self, height):
        # TODO check that target height > body upper atmosphere limit
        # TODO allow different angled orbits
        # TODO generalise for other planets
        # TODO check for control loss (large deviation of angle from target)
        # TODO time acceleration
        self.height = height

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
        if a >= self.height:
            self.throttle = 0
            res = True
        elif a > self.height * 0.9:
            # scale throttle
            self.throttle = (self.height - a) / (self.height * 0.1)
        else:
            self.throttle = 1
        return res

    def do_gravity_turn(self, altitude):
        a = altitude()
        if a < 10000:
            return
        if not self.started_gravity_turn:
            logger.info("Starting gravity turn")
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
        self.v.auto_pilot.target_pitch_and_heading(90, 90)
        self.v.auto_pilot.engage()
        self.v.control.throttle = 1
        time.sleep(1)


class DropStagesAndOpenParachute(Step):
    # TODO enable autopilot, point retrograde
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
        with core.conn.stream(getattr, v.flight(), "mean_altitude") as altitude:
            logger.info("Waiting for parachute altitude")
            while altitude() > 5000:
                time.sleep(1)
            while altitude() > 2500:
                time.sleep(0.1)
            logger.info("Opening parachutes")
            v.control.activate_next_stage()


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
