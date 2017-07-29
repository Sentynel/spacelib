import logging

from ..steps import *

logger = logging.getLogger(__name__)

class Mission:
    """A series of Steps in a mission."""
    steps = []
    def execute(self, v):
        for step, abort in self.steps:
            try:
                step.execute(v)
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
            (DropStagesAndOpenParachute(), None),
            ]
