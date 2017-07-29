import logging
import time

logger = logging.getLogger(__name__)


class FailedStep(Exception):
    pass


class Step:
    """A single step in a mission."""
    def execute(self, v):
        """Run this step of a mission.

        Takes the vessel to run on."""
        raise NotImplementedError


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
                raise FailedStep("altitude decreasing unexpectedly")
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
        while v.flight().surface_altitude > 1500:
            time.sleep(1)

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
