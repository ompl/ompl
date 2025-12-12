import numpy as np
from pathlib import Path
import pandas as pd
import random
import copy
import vamp
from functools import partial
from fire import Fire
from ompl import base as ob
from ompl import geometric as og
import time
# Starting configuration
a = [0., -0.785, 0., -2.356, 0., 1.571, 0.785]

# Goal configuration
b = [2.35, 1., 0., -0.8, 0, 2.5, 0.785]
# Problem specification: a list of sphere centers
problem = [
    [0.55, 0, 0.25],
    [0.35, 0.35, 0.25],
    [0, 0.55, 0.25],
    [-0.55, 0, 0.25],
    [-0.35, -0.35, 0.25],
    [0, -0.55, 0.25],
    [0.35, -0.35, 0.25],
    [0.35, 0.35, 0.8],
    [0, 0.55, 0.8],
    [-0.35, 0.35, 0.8],
    [-0.55, 0, 0.8],
    [-0.35, -0.35, 0.8],
    [0, -0.55, 0.8],
    [0.35, -0.35, 0.8],
    ]

class VampMotionValidator(ob.MotionValidator):
    def __init__(self, si, env: vamp.Environment, dimension: int):
        super().__init__(si)
        self.env = env
        self.dimension = dimension

    def checkMotion(self, s1:ob.RealVectorStateType, s2:ob.RealVectorStateType) -> bool:
        config1 = s1[0:self.dimension]
        config2 = s2[0:self.dimension]
        return vamp.panda.validate_motion(config1, config2, self.env)

def isStateValid(s:ob.RealVectorStateType, env:vamp.Environment, dimension: int) -> bool:
    config = s[0:dimension]
    return vamp.panda.validate(config, env)

def main(
    variation: float = 0.01,
    benchmark: bool = True,
    n_trials: int = 100,
    radius: float = 0.2,
    visualize: bool = False,
    ):

    robot = vamp.panda
    dimension = robot.dimension()
    space = ob.RealVectorStateSpace(dimension)
    bounds = ob.RealVectorBounds(dimension)
    upper_bounds = robot.upper_bounds()
    print(upper_bounds)
    lower_bounds = robot.lower_bounds()
    print(lower_bounds)
    for i in range(dimension):
        bounds.setLow(i, lower_bounds[i])
        bounds.setHigh(i, upper_bounds[i])
    space.setBounds(bounds)
    
    if benchmark:
        random.seed(0)
        np.random.seed(0)

        results = []
        spheres = [np.array(sphere) for sphere in problem]
        for _ in range(n_trials):
            random.shuffle(spheres)
            spheres_copy = copy.deepcopy(spheres)

            e = vamp.Environment()
            for sphere in spheres_copy:
                sphere += np.random.uniform(low = -variation, high = variation, size = (3, ))
                e.add_sphere(vamp.Sphere(sphere, radius))
            planning_start = time.time()
            si = ob.SpaceInformation(space)
            planner = og.RRTConnect(si)
            si.setMotionValidator(VampMotionValidator(si, e, dimension))
            si.setStateValidityChecker(partial(isStateValid, env=e, dimension=dimension))
            ss = og.SimpleSetup(si)
            ss.setPlanner(planner)
            start = si.allocState()
            start[0:dimension] = a
            goal = si.allocState()
            goal[0:dimension] = b
            ss.setStartAndGoalStates(start, goal)
            result = ss.solve(1.0)
            planning_time = time.time() - planning_start
            if result:
                results.append({
                    "planning_time": planning_time,
                })
        df = pd.DataFrame.from_dict(results)
        print(df.describe())


if __name__ == "__main__":
    Fire(main)
