import numpy as np
import random
import copy
import vamp
import vamp.pointcloud
from functools import partial
from fire import Fire
from ompl import base as ob
from ompl import geometric as og
from ompl import tools as ot
from vamp_state_space import VampMotionValidator, VampStateValidityChecker, VampStateSpace

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from viser_visualizer.viser_visualizer import ViserVisualizer

import time
import subprocess
import os
import shutil

POINTS_PER_SPHERE = 300
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


def isStateValid(s:ob.RealVectorStateType, env:vamp.Environment, dimension: int) -> bool:
    config = s[0:dimension]
    return vamp.panda.validate(config, env)


def create_environment(
    variation: float = 0.01,
    radius: float = 0.2,
    use_point_cloud: bool = False,
    points_per_sphere: int = POINTS_PER_SPHERE,
    ):
    """Create a VAMP environment with either spheres or point cloud obstacles.

    Returns (env, spheres, point_cloud) where point_cloud is None for sphere mode.
    """
    spheres = [np.array(sphere) for sphere in problem]
    random.shuffle(spheres)
    spheres_copy = copy.deepcopy(spheres)

    for sphere in spheres_copy:
        sphere += np.random.uniform(low=-variation, high=variation, size=(3,))

    env = vamp.Environment()
    point_cloud = None

    if use_point_cloud:
        point_cloud = np.vstack(
            [
                vamp.pointcloud.sphere_sample_surface(center, radius, points_per_sphere, 0.0)
                for center in spheres_copy
            ]
        ).astype(np.float32)

        r_min, r_max = vamp.panda.min_max_radii()
        filtered_point_cloud_list, elapsed_ns = vamp.filter_pointcloud(point_cloud, r_min, 2, (0,0,0), (-2,-2,-2), (2,2,2), True)
        n_added = env.add_pointcloud(filtered_point_cloud_list, r_min, r_max, 0.01)
    else:
        for sphere in spheres_copy:
            env.add_sphere(vamp.Sphere(sphere, radius))

    return env, spheres, point_cloud


def planOnce(
    variation: float = 0.01,
    radius: float = 0.2,
    use_point_cloud: bool = False,
    points_per_sphere: int = POINTS_PER_SPHERE,
    visualize: bool = False
    ):

    env, spheres, point_cloud = create_environment(
        variation, radius, use_point_cloud, points_per_sphere)

    # Use VAMP's robot module to initialize state space and validations
    robot = vamp.panda
    dimension = robot.dimension()
    space = VampStateSpace(robot=robot)
    si = ob.SpaceInformation(space)

    motion_validator = VampMotionValidator(si=si, env=env, robot=robot)
    state_validity_checker = VampStateValidityChecker(si=si, env=env, robot=robot)

    # Select a planner
    planner = og.RRTConnect(si)

    # Set validators
    si.setMotionValidator(motion_validator)
    si.setStateValidityChecker(state_validity_checker)

    # Build SimpleSetup object
    ss = og.SimpleSetup(si)
    ss.setPlanner(planner)

    # Create start and goal states
    start = si.allocState()
    start[0:dimension] = a
    goal = si.allocState()
    goal[0:dimension] = b
    ss.setStartAndGoalStates(start, goal)

    # Solve
    planning_start = time.time()
    result = ss.solve(5.0)
    if result:
        path = ss.getSolutionPath()
        print(f"Planning time: {time.time() - planning_start}")
        print(f"Generated path with {path.getStateCount()} states")

        if visualize:
            visualizer = ViserVisualizer(robot_name="panda",
                                        robot_dimension=dimension,
                                        port=8080)

            if point_cloud is not None:
                pc_colors = np.tile([204, 102, 51], (point_cloud.shape[0], 1))
                visualizer.add_point_cloud(point_cloud, color=pc_colors)
            else:
                for sphere in spheres:
                    visualizer.add_sphere(sphere, radius, color=(0.8, 0.4, 0.2))

            path.interpolate(50)
            states = path.getStates()
            trajectory = np.array([list(state[0:dimension]) for state in states])
            visualizer.visualize_trajectory(trajectory)
            visualizer.play_until_key_pressed()
    else:
        print(f"No solution found (time: {time.time() - planning_start})")


def planBenchmark(
    variation: float = 0.01,
    radius: float = 0.2,
    use_point_cloud: bool = False,
    points_per_sphere: int = POINTS_PER_SPHERE,
    n_trials: int = 100
    ):

    env, spheres, point_cloud = create_environment(
        variation, radius, use_point_cloud, points_per_sphere)

    # Use VAMP's robot module to initialize state space and validations
    robot = vamp.panda
    dimension = robot.dimension()
    space = VampStateSpace(robot=robot)
    si = ob.SpaceInformation(space)

    motion_validator = VampMotionValidator(si=si, env=env, robot=robot)
    state_validity_checker = VampStateValidityChecker(si=si, env=env, robot=robot)

    # Set validators
    si.setMotionValidator(motion_validator)
    si.setStateValidityChecker(state_validity_checker)

    # Build SimpleSetup object
    ss = og.SimpleSetup(si)

    start = si.allocState()
    start[0:dimension] = a
    goal = si.allocState()
    goal[0:dimension] = b
    ss.setStartAndGoalStates(start, goal)

    # Create Benchmark
    env_label = "PointCloud" if use_point_cloud else "Sphere"
    benchmark = ot.Benchmark(ss, f"Vamp Cage Planning Benchmark ({env_label})")
    benchmark.addPlanner(og.RRTConnect(si))
    benchmark.addPlanner(og.RRT(si))
    benchmark.addPlanner(og.KPIECE1(si))
    benchmark.addPlanner(og.LBKPIECE1(si))

    # Setup Benchmark
    req = ot.Request()
    req.maxTime = 1.0
    req.maxMem = 100.0
    req.runCount = n_trials
    req.displayProgress = True
    benchmark.benchmark(req)

    log_file = "benchmark.log"
    benchmark.saveResultsToFile(log_file)
    db_path = "benchmark.db"
    ot.readBenchmarkLog(db_path, [log_file], moveitformat=False)
    print(f"Database saved to {db_path}")

    try:
        import plannerarena.app
        print(f"Displaying benchmark results from {db_path}")
        plannerarena.app.DATABASE = os.path.abspath(db_path)
        plannerarena.app.run()
    except ImportError:
        print("PlannerArena not found, please install it from https://github.com/ompl/plannerarena")

def main(
    variation: float = 0.01,
    benchmark: bool = False,
    use_point_cloud: bool = False,
    n_trials: int = 100,
    radius: float = 0.2,
    visualize: bool = False
    ):

    if benchmark:
        planBenchmark(variation, radius, use_point_cloud, n_trials=n_trials)
    else:
        planOnce(variation, radius, use_point_cloud, visualize=visualize)
   
        
if __name__ == "__main__":
    Fire(main)