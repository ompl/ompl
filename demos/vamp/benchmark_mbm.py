import pickle
import time
from pathlib import Path
from typing import Union, List
from functools import partial
import sys
from datetime import datetime

from fire import Fire
import vamp
from vamp import pointcloud as vpc
from ompl import base as ob
from ompl import geometric as og
from ompl import tools as ot
from validity import VampMotionValidator, isStateValid

sys.path.insert(0, str(Path(__file__).parent.parent))
from viser_visualizer import ViserVisualizer

def main(
    robot: str = "panda",                  # Robot to plan for
    dataset: str = "problems.pkl",         # Pickled dataset to use
    problem: Union[str, List[str]] = [],   # Problem name or list of problems to evaluate
    n_trials: int = 100,                   # Number of trials to benchmark for each problem x planner
    pointcloud: bool = False,              # Use pointcloud rather than primitive geometry
    samples_per_object: int = 10000,       # If pointcloud, samples per object to use
    filter_radius: float = 0.02,           # Filter radius for pointcloud filtering
    filter_cull: bool = True,              # Cull pointcloud around robot by maximum distance
    planning_time: float = 1.0,            # Planning time limit in seconds
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
    
    vamp_folder = Path(__file__).parent.parent.parent.parent / 'external' / 'vamp'
    problems_dir = vamp_folder / 'resources' / robot
    with open(problems_dir / dataset, 'rb') as f:
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
    
    # Create benchmark output directory with timestamp
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    benchmark_dir = Path("benchmarks") / f"benchmark_{timestamp}"
    benchmark_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"Benchmarking {robot} on first problem of each type with {n_trials} trials...")
    print(f"Results will be saved to: {benchmark_dir}")
    
    # Set this to x plans to do
    planner_constructors = {
        'rrtc': og.RRTConnect,
        'rrt': og.RRT,
        'kpiece1': og.KPIECE1,
        'lbkpiece1': og.LBKPIECE1,
    }
    
    tick = time.perf_counter()
    
    for name, pset in problems['problems'].items():
        if name not in problem:
            continue
        
        print(f"\nProcessing problem type: {name}")
        
        # TODO: For now we use one (the first) problem from each problem type
        first_valid_problem = None
        for i, data in enumerate(pset):
            if data['valid']:
                first_valid_problem = data
                print(f"  Using problem index {i} (first valid problem)")
                break
        
        if first_valid_problem is None:
            print(f"  No valid problems found in {name}, skipping...")
            continue
        
        if pointcloud:
            r_min, r_max = robot_module.min_max_radii()
            (env, original_pc, filtered_pc, filter_time, build_time) = vpc.problem_dict_to_pointcloud(
                robot,
                r_min,
                r_max,
                first_valid_problem,
                samples_per_object,
                filter_radius,
                filter_cull,
            )
            print(f"  Pointcloud: {len(original_pc)} -> {len(filtered_pc)} points")
        else:
            env = vamp.problem_dict_to_vamp(first_valid_problem)
        
        # Setup OMPL problem
        si = ob.SpaceInformation(space)
        si.setMotionValidator(VampMotionValidator(si, env, robot_module, dimension))
        si.setStateValidityChecker(partial(isStateValid, env=env, robot_module=robot_module, dimension=dimension))
        
        ss = og.SimpleSetup(si)
        
        start = si.allocState()
        start[0:dimension] = first_valid_problem['start']
        
        goal = si.allocState()
        goal[0:dimension] = first_valid_problem['goals'][0]
        
        ss.setStartAndGoalStates(start, goal)
        
        benchmark = ot.Benchmark(ss, f"{robot}_{name}_benchmark")
        
        for planner_name, planner_constructor in planner_constructors.items():
            benchmark.addPlanner(planner_constructor(si))
        
        # Configure benchmark request
        req = ot.Request()
        req.maxTime = planning_time
        req.maxMem = 1000.0
        req.runCount = n_trials
        req.displayProgress = True
        
        print(f"  Running benchmark with {n_trials} trials...")
        benchmark.benchmark(req)
        
        # Save results
        log_file = str(benchmark_dir / f"{name}.log")
        db_file = str(benchmark_dir / f"{name}.db")
        
        benchmark.saveResultsToFile(log_file)
        print(f"  Saved log to: {log_file}")
        
        # Convert log to database
        ot.readBenchmarkLog(db_file, [log_file], moveitformat=False)
        print(f"  Saved database to: {db_file}")
        
        total_problems += 1

    tock = time.perf_counter()

    print(f"\n{'='*60}")
    print(f"Benchmark Complete!")
    print(f"{'='*60}")
    print(f"Total problem types benchmarked: {total_problems}")
    print(f"Total time: {(tock - tick):.2f} seconds")
    print(f"Results saved to: {benchmark_dir}")
    print(f"\nTo view results, use plannerarena or load the .db files with OMPL tools")


if __name__ == "__main__":
    Fire(main)
