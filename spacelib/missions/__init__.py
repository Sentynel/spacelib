import logging

from ..steps import *

logger = logging.getLogger(__name__)

class Mission:
    """A series of Steps in a mission."""
    steps = []
    def execute(self, v, interactive=False):
        if interactive:
            self.print_steps()
            s = int(input("Start at which stage? "))
            self.steps = self.steps[s:]
        for step, abort in self.steps:
            try:
                step.execute(v)
                # Turn off autopilot, turn on auto-stabiliser
                v.auto_pilot.disengage()
                v.control.sas = True
            except Exception:
                logger.exception("Step failed")
                v.auto_pilot.disengage()
                v.control.throttle = 0
                if abort:
                    logger.error("Executing abort")
                    return abort.execute(v)
                else:
                    logger.critical("No abort available")
                    logger.critical("¯\_(ツ)_/¯")
                    return

    def print_steps(self):
        print("Mission plan:")
        for i, s in enumerate(self.steps):
            print("{}: {}".format(i, s[0]))


class SuborbitalAbortMission(Mission):
    steps = [
            (AbortEngines(),None),
            (WaitForParachute,None),
            (WaitForLanding,None),
            ]


class SuborbitalMission(Mission):
    steps = [
            (LaunchPrep(),None),
            (Launch(),SuborbitalAbortMission()),
            (GravityTurn(),SuborbitalAbortMission()),
            (WaitForEmpty(),SuborbitalAbortMission()),
            (WaitForParachute(),None),
            (WaitForLanding(),None),
            ]


class OrbitalMission(Mission):
    steps = [
            (LaunchToApoapsis(100000), DropStagesAndOpenParachute()),
            (Circularise("apoapsis"), DropStagesAndOpenParachute()),
            # TODO orbital abort
            (ExecuteNode(), DropStagesAndOpenParachute()),
            (ControllerConfirm(), None),
            (ChangePeriapsis(60, 40000), None),
            (ExecuteNode(), None),
            (DropStagesAndOpenParachute(), None),
            ]
