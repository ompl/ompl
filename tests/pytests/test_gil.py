"""Python callbacks called from OMPL's C++ threads must not deadlock or crash."""

import faulthandler
import gc
import os
import random
import subprocess
import sys
import time

import pytest

from ompl import base as ob
from ompl import geometric as og
from ompl import util as ou

TIMEOUT = 60  # seconds per scenario; a correct run takes well under one
SOLVE_TIME = 10.0  # planners return as soon as they solve, so this only bounds failures


def make_problem():
    space = ob.RealVectorStateSpace(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0)
    bounds.setHigh(1)
    space.setBounds(bounds)
    si = ob.SpaceInformation(space)
    # Python validity checker: every planner thread that checks states calls into Python.
    si.setStateValidityChecker(lambda s: (s[0] - 0.5) ** 2 + (s[1] - 0.5) ** 2 > 0.09)
    si.setup()
    pdef = ob.ProblemDefinition(si)
    start = si.allocState()
    start[0] = start[1] = 0.1
    pdef.addStartState(start)
    return si, pdef


def sample_near_goal(state):
    state[0] = 0.9 + random.uniform(-0.05, 0.05)
    state[1] = 0.9 + random.uniform(-0.05, 0.05)


def endless_sampler(gls, state):
    sample_near_goal(state)
    return True  # never stops by itself: only stopSampling() or destruction ends it


def solve_and_check(planner, pdef):
    planner.setProblemDefinition(pdef)
    planner.setup()
    assert planner.solve(SOLVE_TIME)


def prm_multiple_goals():
    # With several goals, PRM's solution-checking thread samples goals and validates them in Python.
    si, pdef = make_problem()
    goals = ob.GoalStates(si)
    for _ in range(50):
        g = si.allocState()
        sample_near_goal(g)
        goals.addState(g)
    pdef.setGoal(goals)
    for _ in range(5):
        pdef.clearSolutionPaths()
        solve_and_check(og.PRM(si), pdef)


def goal_lazy_samples(planner_type):
    # The sampler runs on the goal's own thread. Solving also checks that the sampler makes
    # progress while solve() blocks, i.e. that the GIL is released.
    for _ in range(5):
        si, pdef = make_problem()
        pdef.setGoal(ob.GoalLazySamples(si, endless_sampler))
        solve_and_check(planner_type(si), pdef)


def goal_lazy_samples_simple_setup():
    si, _ = make_problem()
    ss = og.SimpleSetup(si)
    start = si.allocState()
    start[0] = start[1] = 0.1
    ss.getProblemDefinition().addStartState(start)
    for _ in range(5):
        ss.setGoal(ob.GoalLazySamples(si, endless_sampler))
        assert ss.solve(SOLVE_TIME)
        ss.clear()


def goal_lazy_samples_destroyed_while_sampling():
    # Destruction must stop the sampler without deadlocking, and never call it with the dying object.
    si, _ = make_problem()
    calls = []

    def counting_sampler(gls, state):
        calls.append(1)  # don't store gls: that would keep the object alive
        return endless_sampler(gls, state)

    for _ in range(20):
        gls = ob.GoalLazySamples(si, counting_sampler)
        time.sleep(0.005)  # let the sampler get going, so destruction races with a call
        del gls
        gc.collect()
        stopped_at = len(calls)
        time.sleep(0.01)
        assert len(calls) == stopped_at, (
            "sampler still called after the goal was destroyed"
        )
    assert calls, "sampler never ran"


def termination_condition_fn():
    # solve(fn, checkInterval) polls the Python fn from a separate C++ thread.
    si, pdef = make_problem()
    state = si.allocState()
    state[0] = state[1] = 0.9
    goal = ob.GoalState(si)
    goal.setState(state)
    pdef.setGoal(goal)
    for _ in range(5):
        pdef.clearSolutionPaths()
        planner = og.PRM(si)
        planner.setProblemDefinition(pdef)
        planner.setup()
        deadline = time.monotonic() + SOLVE_TIME
        assert planner.solve(lambda: time.monotonic() > deadline, 0.01)


SCENARIOS = {
    "prm_multiple_goals": prm_multiple_goals,
    "goal_lazy_samples_prm": lambda: goal_lazy_samples(og.PRM),
    "goal_lazy_samples_rrtconnect": lambda: goal_lazy_samples(og.RRTConnect),
    "goal_lazy_samples_simple_setup": goal_lazy_samples_simple_setup,
    "goal_lazy_samples_destroyed_while_sampling": goal_lazy_samples_destroyed_while_sampling,
    "termination_condition_fn": termination_condition_fn,
}


@pytest.mark.parametrize("scenario", SCENARIOS)
def test_no_deadlock_or_crash(scenario):
    env = {**os.environ, "PYTHONPATH": os.pathsep.join(sys.path)}
    try:
        proc = subprocess.run(
            [sys.executable, __file__, scenario],
            capture_output=True,
            text=True,
            env=env,
            timeout=TIMEOUT + 30,
        )
    except subprocess.TimeoutExpired:
        pytest.fail(f"{scenario}: hung and did not respond to its watchdog")
    # On a hang the child's watchdog dumps all thread stacks to stderr and exits with code 1.
    assert proc.returncode == 0, (
        f"{scenario} failed (exit code {proc.returncode}):\n{proc.stderr[-4000:]}"
    )


if __name__ == "__main__":
    faulthandler.dump_traceback_later(TIMEOUT, exit=True)
    ou.setLogLevel(ou.LogLevel.LOG_WARN)
    random.seed(0)
    SCENARIOS[sys.argv[1]]()
