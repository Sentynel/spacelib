import json
import logging

import krpc

from .. import core
from .. import steps
from ..steps import *

logger = logging.getLogger(__name__)

class Mission:
    """A series of Steps in a mission."""
    steps = []
    def __init__(self, steps=None):
        if steps is not None:
            self.steps = steps

    def execute(self, interactive=False):
        if interactive:
            self.print_steps()
            s = int(input("Start at which stage? "))
            self.steps = self.steps[s:]
        for step, abort in self.steps:
            try:
                v = core.conn.space_center.active_vessel
            except krpc.error.RPCError:
                v = None
            try:
                step.execute(v)
                if v:
                    # Turn off autopilot, turn on auto-stabiliser
                    v.auto_pilot.disengage()
                    v.control.sas = True
                    v.control.throttle = 0
            except Exception:
                logger.exception("Step failed")
                if v:
                    v.auto_pilot.disengage()
                    v.control.throttle = 0
                    v.control.sas = True
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
            (ChangeApsis(60, 40000), None),
            (ExecuteNode(), None),
            (DropStagesAndOpenParachute(), None),
            ]


def load_mission_from_file(fn):
    """Load mission from a JSON file.

    Expected format looks like this:
    [
        [StepName, [Arg1, Arg2], AbortName, [Arg1, Arg2]],
    ]
    All sections after the StepName are optional.
    """
    with open(fn) as f:
        data = json.load(f)
    return load_mission(data)

def load_mission_from_json(s):
    data = json.loads(s)
    return load_mission(data)

def load_mission(data):
    plan = []
    for items in data:
        step = items[0]
        if len(items) > 1:
            args = items[1]
        else:
            args = []
        if len(items) > 2:
            abort = items[2]
        else:
            abort = None
        if len(items) > 3:
            abort_args = items[3]
        else:
            abort_args = []
        line = (
                getattr(steps, step)(*args),
                getattr(steps, abort)(*abort_args) if abort else abort,
                )
        plan.append(line)
    return Mission(plan)

def mission_repl():
    try:
        while True:
            s = input("Mission to execute:\n")
            m = load_mission_from_json(s)
            m.execute()
    except KeyboardInterrupt:
        logger.info("Interrupted")
