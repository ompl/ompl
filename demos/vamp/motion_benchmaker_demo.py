import pickle
import time
from tabulate import tabulate
from tqdm import tqdm
from pathlib import Path
import pandas as pd
from typing import Union, List
from functools import partial
import sys
from datetime import datetime
import subprocess
import numpy as np

from fire import Fire
import vamp
from vamp import pointcloud as vpc
from ompl import base as ob
from ompl import geometric as og
from ompl import tools as ot

from vamp_state_space import VampStateSpace, VampMotionValidator, VampStateValidityChecker

sys.path.insert(0, str(Path(__file__).parent.parent))
from viser_visualizer import ViserVisualizer

def main(
    robot: str = "panda",                  # Robot to plan for
    planner: str = "rrtc",                 # Planner name to use (e.g., 'rrtc', 'prm', 'rrt')
    dataset: str = "problems.pkl",         # Pickled dataset to use
    problem: Union[str, List[str]] = [],   # Problem name or list of problems to evaluate
    trials: int = 1,                       # Number of trials to evaluate each instance
    print_failures: bool = False,          # Print out failures and invalid problems
    visualize: bool = False,              # Visualize solutions using Viser (any key=next, q=disable, w=skip problem set)
    pointcloud: bool = False,              # Use pointcloud rather than primitive geometry
    samples_per_object: int = 10000,       # If pointcloud, samples per object to use
    filter_radius: float = 0.02,           # Filter radius for pointcloud filtering
    filter_cull: bool = True,              # Cull pointcloud around robot by maximum distance
    planning_time: float = 1.0,            # Planning time limit in seconds
    benchmark: int = 0,                   # Run OMPL benchmark for n trials instead of 700 problems
    n_trials: int = 100,                   # Number of trials for benchmarking
    **kwargs,
    ):

    if robot not in vamp.robots:
        raise RuntimeError(f"Robot {robot} does not exist in VAMP!")

    # Get robot module
    robot_module = getattr(vamp, robot)
    dimension = robot_module.dimension()
    
    # Setup OMPL state space
    space = ob.RealVectorStateSpace(dimension)
    bounds = ob.RealVectorBounds(dimension)
    upper_bounds = robot_module.upper_bounds()
    lower_bounds = robot_module.lower_bounds()
    for i in range(dimension):
        bounds.setLow(i, lower_bounds[i])
        bounds.setHigh(i, upper_bounds[i])
    space.setBounds(bounds)
    
    # Map planner names to OMPL planners
    planner_map = {
        'rrtc': og.RRTConnect,
        'rrt': og.RRT,
        'prm': og.PRM,
        'rrtstar': og.RRTstar,
        'bitstar': og.BITstar,
    }
    
    if planner.lower() not in planner_map:
        raise RuntimeError(f"Planner {planner} not recognized. Available: {list(planner_map.keys())}")
    
    planner_class = planner_map[planner.lower()]
    vamp_folder = Path(__file__).parent.parent.parent / 'external' / 'vamp'
    problems_dir = vamp_folder / 'resources' / robot
    pickle_path = problems_dir / dataset
    
    # Check if pickle file exists, generate if not
    if not pickle_path.exists():
        print(f"Pickle file not found at {pickle_path}, generating from tar.bz2...")
        script_path = vamp_folder / 'resources' / 'problem_tar_to_pkl_json.py'
        result = subprocess.run([sys.executable, str(script_path), f'--robot={robot}'], check=True)
        if result.returncode != 0:
            raise RuntimeError(f"Failed to generate pickle file using {script_path}")
        print(f"Successfully generated pickle file")
    
    with open(pickle_path, 'rb') as f:
        problems = pickle.load(f)

    problem_names = list(problems['problems'].keys())
    if isinstance(problem, str):
        problem = [problem]

    if not problem:
        problem = problem_names
    else:
        for problem_name in problem:
            if problem_name not in problem_names:
                raise RuntimeError(
                    f"Problem `{problem_name}` not available! Available problems: {problem_names}"
                    )

    total_problems = 0
    valid_problems = 0
    failed_problems = 0
    
    # Initialize visualizer if requested
    vis = None
    visualize_enabled = visualize
    skip_to_next_problem_set = False
    if visualize:
        vis = ViserVisualizer(robot_name=robot, robot_dimension=dimension)
        # vis.add_grid()
    
    if benchmark:    
        # Create benchmark output directory with timestamp
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        benchmark_dir = Path("benchmarks") / f"benchmark_{timestamp}"
        benchmark_dir.mkdir(parents=True, exist_ok=True)
        
        print(f"Benchmarking {robot} on first problem of each type with {n_trials} trials...")
        print(f"Results will be saved to: {benchmark_dir}")
        
        # Set this to x plans to do
        planner_constructors = [
            og.RRTConnect,
            og.PRM,
            og.KPIECE1,
            og.LBKPIECE1,
        ]
        
    tick = time.perf_counter()
    results = []
    
    print(f"Evaluating {planner_class.__name__} on problems: {problems['problems'].keys()} with robot {robot}...")
    
    for name, pset in problems['problems'].items():
        if name not in problem:
            continue
        
        # Reset skip flag for new problem set
        skip_to_next_problem_set = False

        failures = []
        invalids = []
        print(f"Evaluating {robot} on {name}: ")
        progress_bar = tqdm(total=len(pset), desc=f"Processing {name}")
        for i, data in enumerate(pset):
            total_problems += 1
            skip_pset = False
            progress_bar.update(1)

            if not data['valid']:
                invalids.append(i)
                continue

            valid_problems += 1

            if pointcloud:
                r_min, r_max = robot_module.min_max_radii()
                (env, original_pc, filtered_pc, filter_time, build_time) = vpc.problem_dict_to_pointcloud(
                    robot,
                    r_min,
                    r_max,
                    data,
                    samples_per_object,
                    filter_radius,
                    filter_cull,
                    )

                pointcloud_results = {
                    'original_pointcloud_size': len(original_pc),
                    'filtered_pointcloud_size': len(filtered_pc),
                    'filter_time': pd.Timedelta(nanoseconds = filter_time),
                    'capt_build_time': pd.Timedelta(nanoseconds = build_time)
                    }
            else:
                env = vamp.problem_dict_to_vamp(data)

            for trial in range(trials):
                # Setup OMPL problem
                dimension = robot_module.dimension()
                space = VampStateSpace(robot=robot_module)
                si = ob.SpaceInformation(space)
                
                motion_validator = VampMotionValidator(si = si, env = env, robot = robot_module)
                state_validity_checker = VampStateValidityChecker(si = si, env = env, robot = robot_module)
                
                # Set validators
                si.setMotionValidator(motion_validator)
                si.setStateValidityChecker(state_validity_checker)
                
                ss = og.SimpleSetup(si)
                ompl_planner = planner_class(si)
                ss.setPlanner(ompl_planner)
                
                # Set start state
                start = si.allocState()
                start[0:dimension] = data['start']
                
                # Set goal state (use first goal)
                goal = si.allocState()
                goal[0:dimension] = data['goals'][0]
                
                ss.setStartAndGoalStates(start, goal)
                
                # Benchmark
                if benchmark:
                    benchmark_name = f"{robot}_{name}"
                    ompl_benchmark = ot.Benchmark(ss, benchmark_name)
            
                    for planner_constructor in planner_constructors:
                        ompl_benchmark.addPlanner(planner_constructor(si))
                    
                    # Configure benchmark request
                    req = ot.Request()
                    req.maxTime = planning_time
                    req.maxMem = 1000.0
                    req.runCount = n_trials
                    req.displayProgress = True
                    
                    print(f"  Running benchmark with {n_trials} trials...")
                    ompl_benchmark.benchmark(req)
                    
                    # Save results
                    file = str(f"vamp_mbm_python")
                    log_file = file + ".log"
                    db_file = file + ".db"
                    
                    ompl_benchmark.saveResultsToFile(log_file)
                    print(f"  Saved log to: {log_file}")
                    
                    # Convert log to database
                    ot.readBenchmarkLog(db_file, [log_file], moveitformat=False)
                    print(f"  Saved database to: {db_file}")
                    # only run one per problem
                    skip_pset = True
                    break
                # Single Plan
                planning_start = time.perf_counter()
                result = ss.solve(planning_time)
                planning_elapsed = time.perf_counter() - planning_start
                
                if not ss.haveExactSolutionPath():
                    failures.append(i)
                    print(f"Failed to find exact solution for problem {i}")
                    break

                # Get path
                path = ss.getSolutionPath()
                initial_cost = path.length()
                
                # Simplify
                simplify_start = time.perf_counter()
                ss.simplifySolution()
                simplify_elapsed = time.perf_counter() - simplify_start
                
                simplified_path = ss.getSolutionPath()
                simplified_cost = simplified_path.length()
                
                trial_result = {
                    'planning_time': pd.Timedelta(microseconds=planning_elapsed * 1e6),
                    'simplification_time': pd.Timedelta(microseconds=simplify_elapsed * 1e6),
                    'total_time': pd.Timedelta(microseconds=(planning_elapsed + simplify_elapsed) * 1e6),
                    'initial_path_cost': initial_cost,
                    'simplified_path_cost': simplified_cost,
                    'planning_iterations': 0,  # OMPL doesn't directly expose this
                    'avg_time_per_iteration': 0.0,
                }
                
                if pointcloud:
                    trial_result.update(pointcloud_results)

                results.append(trial_result)
                
                # Visualization
                if visualize_enabled and vis is not None:
                    # Check if we should skip to next problem set
                    if skip_to_next_problem_set:
                        continue
                    # Clear previous visualization
                    vis.reset()
                    # vis.add_grid()
                    
                    # Load environment obstacles
                    if pointcloud:
                        original_pc = np.array(original_pc)
                        z_values = original_pc[:, 2]
                        z_min, z_max = z_values.min(), z_values.max()
                        z_range = z_max - z_min
                        
                        # Define color range: start at 20% and end at 80% of z-range
                        color_start = z_min + 0.2 * z_range
                        color_end = z_min + 1.0 * z_range
                        
                        # Clamp and normalize z values to [0, 1] within the color range
                        z_normalized = np.clip((z_values - color_start) / (color_end - color_start), 0, 1)
                        
                        # Smooth rainbow colormap: red -> yellow -> green -> cyan -> blue
                        colors = np.zeros((len(original_pc), 3))
                        colors[:, 0] = np.maximum(0, 1 - 2 * np.abs(z_normalized - 0.25))  # Red
                        colors[:, 1] = np.maximum(0, 1 - 2 * np.abs(z_normalized - 0.5))   # Green
                        colors[:, 2] = np.maximum(0, 1 - 2 * np.abs(z_normalized - 0.75))  # Blue
                        vis.add_point_cloud(original_pc, color=colors, point_size=0.01)
                    else:
                        vis.load_mbm_environment(data, padding=0.0, color=(0.8, 0.4, 0.2, 0.75))
                    
                    # Convert path to numpy array
                    simplified_path.interpolate(150)
                    states = simplified_path.getStates()
                    trajectory = np.array([list(state[0:dimension]) for state in states])
                    # interpolate
                    
                    
                    vis.visualize_trajectory(trajectory)
                    
                    print(f"\nVisualizing problem {name} [{i}] - Press any key to continue, 'q' to disable viz, 'w' to skip to next problem set")
                    key = vis.play_until_key_pressed(key='any', dt=1/60)
                    
                    if key == 'q':
                        print("Visualization disabled for remaining problems")
                        visualize_enabled = False
                    elif key == 'w':
                        print(f"Skipping to next problem set...")
                        skip_to_next_problem_set = True
                        break 
            
            if skip_pset:
                break
        
        progress_bar.close()
        
        failed_problems += len(failures)

        if print_failures:
            if invalids:
                print(f"  Invalid problems: {invalids}")

            if failures:
                print(f"  Failed on {failures}")
                
    
    if benchmark > 0:
        print(f"All benchmarks saved in {benchmark_dir}")
        exit(0)
    
    tock = time.perf_counter()

    df = pd.DataFrame.from_dict(results)

    # Convert to microseconds
    df["planning_time"] = df["planning_time"].dt.microseconds
    df["simplification_time"] = df["simplification_time"].dt.microseconds
    df["avg_time_per_iteration"] = df["planning_iterations"] / df["planning_time"]

    # Pointcloud data
    if pointcloud:
        df["total_build_and_plan_time"] = df["total_time"] + df["filter_time"] + df["capt_build_time"]
        df["filter_time"] = df["filter_time"].dt.microseconds / 1e3
        df["capt_build_time"] = df["capt_build_time"].dt.microseconds / 1e3
        df["total_build_and_plan_time"] = df["total_build_and_plan_time"].dt.microseconds / 1e3

    df["total_time"] = df["total_time"].dt.microseconds

    # Get summary statistics
    time_stats = df[[
        "planning_time",
        "simplification_time",
        "total_time",
        "planning_iterations",
        "avg_time_per_iteration",
        ]].describe(percentiles = [0.25, 0.5, 0.75, 0.95])
    time_stats.drop(index = ["count"], inplace = True)

    cost_stats = df[[
        "initial_path_cost",
        "simplified_path_cost",
        ]].describe(percentiles = [0.25, 0.5, 0.75, 0.95])
    cost_stats.drop(index = ["count"], inplace = True)

    if pointcloud:
        pointcloud_stats = df[[
            "filter_time",
            "capt_build_time",
            "total_build_and_plan_time",
            ]].describe(percentiles = [0.25, 0.5, 0.75, 0.95])
        pointcloud_stats.drop(index = ["count"], inplace = True)

    print()
    print(
        tabulate(
            time_stats,
            headers = [
                'Planning Time (μs)',
                'Simplification Time (μs)',
                'Total Time (μs)',
                'Planning Iters.',
                'Time per Iter. (μs)',
                ],
            tablefmt = 'github'
            )
        )

    print(
        tabulate(
            cost_stats, headers = [
                ' Initial Cost (L2)',
                '    Simplified Cost (L2)',
                ], tablefmt = 'github'
            )
        )

    if pointcloud:
        print(
            tabulate(
                pointcloud_stats,
                headers = [
                    '  Filter Time (ms)',
                    '    CAPT Build Time (ms)',
                    'Total Time (ms)',
                    ],
                tablefmt = 'github'
                )
            )

    print(
        f"Solved / Valid / Total # Problems: {valid_problems - failed_problems} / {valid_problems} / {total_problems}"
        )
    print(f"Completed all problems in {df['total_time'].sum() / 1000:.3f} milliseconds")
    print(f"Total time including Python overhead: {(tock - tick) * 1000:.3f} milliseconds")


if __name__ == "__main__":
    Fire(main)
