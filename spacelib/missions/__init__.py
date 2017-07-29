import logging

from ..steps import *


class Mission:
    """A series of Steps in a mission."""
    def __init__(self, steps):
        self.steps = steps

    def execute(self, v):
        for step, abort in self.steps:
            try:
                step().execute(v)
            except Exception:
                logging.exception("Step failed")
                if abort:
                    return abort.execute()
                else:
                    logging.critical("No abort available")
                    logging.critical("¯\_(ツ)_/¯")
                    return


class SuborbitalMission(Mission):
    def __init__(self):
        super().__init__([
            (LaunchPrep,None),
            (Launch,SuborbitalAbortMission),
            (GravityTurn,SuborbitalAbortMission),
            (WaitForEmpty,SuborbitalAbortMission),
            (WaitForParachute,None),
            (WaitForLanding,None),
                ])

class SuborbitalAbortMission(Mission):
    def __init__(self):
        super().__init__([
            (AbortEngines,None),
            (WaitForParachute,None),
            (WaitForLanding,None),
            ])
