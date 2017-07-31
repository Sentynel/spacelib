import logging
import math
import time

from .. import core
from ..util import vector_angle

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


class PrepareVessel(Step):
    def __init__(self, name, vessel_type):
        if vessel_type == "SPH":
            self.site = "Runway"
        elif vessel_type == "VAB":
            self.site = "LaunchPad"
        else:
            raise StepFailed("vessel_type must be SPH or VAB")
        self.name = name
        self.vessel_type = vessel_type

    def __str__(self):
        return "Prepare {} for launch".format(self.name)

    def execute(self, v):
        core.conn.space_center.launch_vessel(self.vessel_type, self.name, self.site)
        time.sleep(1)


class StagedStep(Step):
    def execute(self, v):
        self.v = v
        self.solid_fuel_stream = None
        self.liquid_fuel_stream = None

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



class LaunchToApoapsis(StagedStep):
    def __init__(self, height, inclination=0):
        # TODO check that target height > body upper atmosphere limit
        # TODO generalise for other planets
        # TODO check for control loss (large deviation of angle from target)
        # TODO time acceleration
        # TODO autopilot that can actually fly straight
        # TODO check for not making our target apoapsis
        # TODO adjust throttle to burn to apoapsis in gravity turn
        # TODO make gravity turn get the pitch from gravity but the heading from
        # the desired inclination, or maybe set the initial heading in the orbital
        # frame somehow
        self.height = height
        self.inclination = inclination

    def __str__(self):
        return "Launch to {} with inclination {}".format(self.height, self.inclination)

    def execute(self, v):
        logger.info("Begun launch step")
        super().execute(v)
        # Set up needed telemetry
        c = core.conn
        self.throttle = 1
        self.q_limit = 1
        self.gravity_turn_stage = 0
        self.last_altitude = 0
        self.descending_count = 0
        # a frame aligned with the autopilot's frame, but measuring velocity
        # relative to ground
        vel_frame = core.conn.space_center.ReferenceFrame.create_hybrid(
                position=v.orbit.body.reference_frame,
                rotation=v.surface_reference_frame)
        try:
            with c.stream(getattr, v.flight(), "mean_altitude") as altitude, \
                    c.stream(getattr, v.orbit, "apoapsis_altitude") as apoapsis, \
                    c.stream(getattr, v.flight(), "dynamic_pressure") as q, \
                    c.stream(getattr, v.flight(vel_frame), "velocity") as velocity, \
                    c.stream(getattr, v.auto_pilot, "target_direction") as ap_target:
                self.prep_launch()
                logger.info("Launch!")
                v.control.activate_next_stage()
                while True:
                    self.do_failure_check(altitude)
                    self.do_staging()
                    on_target = self.do_apoapsis_check(apoapsis)
                    self.do_gravity_turn(altitude, velocity, ap_target)
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

    def do_gravity_turn(self, altitude, velocity, ap_target):
        PITCHOVER_TARGET = 10
        START_TURN_ALT = 5000
        START_BLEND_ALT = 15000
        END_BLEND_ALT = 20000
        a = altitude()
        if a < START_TURN_ALT:
            return
        if not self.gravity_turn_stage:
            logger.info("Pitch over to start gravity turn")
            # Do pitchover in planet reference frame
            ap = self.v.auto_pilot
            ap.target_pitch_and_heading(90 - PITCHOVER_TARGET, 90 + self.inclination)
            self.gravity_turn_stage = 1
            time.sleep(0.1)
        # wait for velocity vector to match our steering
        if self.gravity_turn_stage == 1 and vector_angle(velocity(), ap_target()) < 1:
            logger.info("Following gravity turn")
            self.gravity_turn_stage = 2
            # follow velocity vector
            ap = self.v.auto_pilot
            ap.reference_frame = self.v.surface_velocity_reference_frame
            time.sleep(0.1)
            ap.target_direction = (0, 1, 0)
        if self.gravity_turn_stage == 2 and a > START_BLEND_ALT:
            logger.info("Reference frame transition to orbital")
            self.gravity_turn_stage = 3
        if self.gravity_turn_stage == 3 and a > END_BLEND_ALT:
            logger.info("Reference frame transition complete")
            self.v.auto_pilot.reference_frame = self.v.orbital_reference_frame
            self.v.auto_pilot.target_direction = (0, 1, 0)
            self.gravity_turn_stage = 4
        if self.gravity_turn_stage == 3:
            # transition to orbital reference frame
            # get orbital velocity direction vector in our frame
            final_vec = core.conn.space_center.transform_direction((0,1,0), self.v.orbital_reference_frame,
                    self.v.surface_velocity_reference_frame)
            initial_vec = (0, 1, 0)
            # blend them from 15000 to 20000 metres
            percentage = (a - START_BLEND_ALT) / (END_BLEND_ALT - START_BLEND_ALT)
            dir_vec = tuple((final_vec[i] - initial_vec[i]) * percentage + initial_vec[i] for i in range(3))
            self.v.auto_pilot.target_direction = dir_vec

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
        self.v.control.sas = False
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


class ChangeApsis(OrbitalAdjustment):
    def __init__(self, when, height):
        super().__init__(when, 0)
        self.height = height

    def __str__(self):
        return "Adjust apsis to {} at {}".format(self.height, self.when)

    def execute(self, v):
        logger.info("Adding change apsis node")
        # treat the supplied value as relative to the surface
        self.height += v.orbit.body.equatorial_radius
        if self.when == "apoapsis":
            self.semi_major = (v.orbit.apoapsis + self.height) / 2
        elif self.when == "periapsis":
            self.semi_major = (v.orbit.periapsis + self.height) / 2
        elif isinstance(self.when, int):
            # in this case, the burn point becomes one new apsis and the target
            # the other
            anomaly = v.orbit.true_anomaly_at_ut(core.conn.space_center.ut + self.when)
            r = v.orbit.radius_at_true_anomaly(anomaly)
            self.semi_major = (r + self.height) / 2
        else:
            raise StepFailed("unknown target time")
        super().execute(v)


class ExecuteNode(StagedStep):
    # TODO detect large deviation from burn vector and cut throttle until it's back
    # TODO take staging into account with burn time calculations

    def __str__(self):
        return "Execute next node"

    def execute(self, v):
        logger.info("Orientating for node execution")
        super().execute(v)
        if not v.control.nodes:
            raise StepFailed("no node to execute")
        node = v.control.nodes[0]
        v.control.sas = False
        ap = v.auto_pilot
        ap.reference_frame = node.reference_frame
        ap.target_direction = node.burn_vector(node.reference_frame)
        ap.engage()
        time.sleep(0.1)
        with core.conn.stream(getattr, ap, "error") as error:
            while error() > 5:
                time.sleep(1)
        ap.wait()

        logger.info("Warping to node burn")
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
        try:
            with core.conn.stream(getattr, node, "remaining_delta_v") as remaining_burn, \
                    core.conn.stream(node.remaining_burn_vector, node.reference_frame) as vector:
                v.control.throttle = 1
                while remaining_burn() > 0.5:
                    self.do_staging()
                    ap.target_direction = vector()
                    t_rem = (burn_time * remaining_burn() / dv)
                    if remaining_burn() < 1:
                        v.control.throttle = 0.05
                    # less than ten seconds of burn time remaining, start throttling back
                    # and increase precision
                    elif t_rem < 5:
                        v.control.throttle = (t_rem / 6) + 0.1
                    else:
                        time.sleep(0.1)
                v.control.throttle = 0
                ap.disengage()
            node.remove()
        finally:
            self.close_streams()
        logger.info("Done")


class ControllerConfirm(Step):
    def __str__(self):
        return "Wait for controller confirmation"

    def execute(self, v):
        input("Press enter to continue mission.")


class DropStages(Step):
    def __str__(self):
        return "Dropping all stages."

    def execute(self, v):
        v.control.throttle = 0
        while v.control.current_stage > 0:
            time.sleep(0.1)
            v.control.activate_next_stage()
    
    
class DropStagesAndOpenParachute(Step):
    # TODO warp to atmosphere
    def __str__(self):
        return "Wait for descent and open parachutes"

    def execute(self, v):
        logger.info("Begun parachute step")
        v.control.sas = False
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
        ap.reference_frame = v.surface_velocity_reference_frame
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


class DeployAntennae(Step):
    def __str__(self):
        return "Deploy antennae"

    def execute(self, v):
        logger.info("Deploying antennae")
        for a in v.parts.antennas:
            if a.deployable:
                a.deployed = True
        time.sleep(1)


class OpenPanels(Step):
    def __str__(self):
        return "Open solar panels"

    def execute(self, v):
        logger.info("Opening solar panels")
        for p in v.parts.solar_panels:
            if p.deployable:
                p.deployed = True
        time.sleep(1)


class JettisonFairings(Step):
    def __str__(self):
        return "Jettison fairings"

    def execute(self, v):
        logger.info("Jettisoning fairings")
        for f in v.parts.fairings:
            try:
                if not f.jettisoned:
                    f.jettison()
            except Exception:
                logger.exception("Jettisoning fairing failed")
        time.sleep(1)


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
