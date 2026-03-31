# Motion Planning for Realistic Robot Arms with VAMP {#vampPlanningTutorial}

[TOC]

This tutorial shows how to use [VAMP](https://github.com/KavrakiLab/vamp) (Vector-Accelerated Motion Planning) with OMPL for robot motion planning. VAMP provides SIMD-accelerated collision checking and forward kinematics, processing multiple robot configurations simultaneously using vectorized instructions. By plugging VAMP into OMPL's planning framework, you get the flexibility of OMPL's planner library with the speed of VAMP's collision checking.

VAMP ships with built-in support for several robots including the Franka Panda, UR5, and Fetch. In this tutorial we will plan for a 7-DOF Panda arm navigating around sphere obstacles. In this tutorial we use the Python bindings for OMPL. There is also a pure C++ version of this demo; see [VAMPPlanning.cpp](VAMPPlanning_8cpp_source.html).

<div class="row justify-content-center"><div class="col-md-8 col-sm-8"><img src="images/vamp_sphere_cage.gif" class="img-fluid"><br><b>The Panda arm inside a cage of 14 sphere obstacles.</b></div></div>

## Prerequisites

Install the required Python packages:

~~~{.sh}
pip install -r demos/vamp/requirements.txt
~~~

You will also need the OMPL Python bindings installed. See the [installation instructions](installation.html) for details.

## Bridging VAMP and OMPL

OMPL is designed to be agnostic to the robot and environment representation. To use VAMP's collision checking with OMPL, we need to implement three bridge classes that translate between OMPL's state representation and VAMP's configuration representation.

### State Space

First, we create a state space that uses VAMP's robot module to define the joint limits. This is a simple wrapper around ompl::base::RealVectorStateSpace that reads bounds directly from the VAMP robot:

~~~{.py}
from ompl import base as ob
import vamp

class VampStateSpace(ob.RealVectorStateSpace):
    def __init__(self, robot: vamp.robot):
        super().__init__(robot.dimension())
        self.robot = robot
        self.dimension = robot.dimension()
        bounds = ob.RealVectorBounds(self.dimension)
        upper_bounds = robot.upper_bounds()
        lower_bounds = robot.lower_bounds()

        for i in range(self.dimension):
            bounds.setLow(i, lower_bounds[i])
            bounds.setHigh(i, upper_bounds[i])
        self.setBounds(bounds)
~~~

The `robot.dimension()` call returns the number of joints (7 for the Panda), and `robot.lower_bounds()` / `robot.upper_bounds()` return the joint limits. This is all OMPL needs to sample and interpolate in the configuration space.

### State Validity Checker

Next, we implement a state validity checker that delegates to VAMP's `robot.validate()` function. This checks whether a single configuration is collision-free against the environment:

~~~{.py}
class VampStateValidityChecker(ob.StateValidityChecker):
    def __init__(self, si: ob.SpaceInformation, env: vamp.Environment, robot: vamp.robot):
        super().__init__(si)
        self.env = env
        self.robot = robot
        self.dimension = robot.dimension()

    def isValid(self, state: ob.State) -> bool:
        config = state[0:self.dimension]
        return self.robot.validate(config, self.env)
~~~

The key line is `self.robot.validate(config, self.env)`, which uses VAMP's vectorized collision checking to test the configuration against all obstacles in the environment.

### Motion Validator

Finally, we implement a motion validator that checks whether an entire straight-line motion between two configurations is collision-free. VAMP provides `robot.validate_motion()` for this purpose, which internally checks configurations along the path at an appropriate resolution:

~~~{.py}
class VampMotionValidator(ob.MotionValidator):
    def __init__(self, si: ob.SpaceInformation, env: vamp.Environment, robot: vamp.robot):
        super().__init__(si)
        self.env = env
        self.robot = robot
        self.dimension = robot.dimension()

    def checkMotion(self, s1: ob.State, s2: ob.State) -> bool:
        config1 = s1[0:self.dimension]
        config2 = s2[0:self.dimension]
        return self.robot.validate_motion(config1, config2, self.env)
~~~

> **Note:** By providing a custom ompl::base::MotionValidator, we bypass OMPL's default ompl::base::DiscreteMotionValidator, which would otherwise call `isValid()` at discrete steps along the motion. VAMP's `validate_motion()` handles the entire edge check internally in a single call.

These three classes are provided in the `vamp_state_space.py` module included with the demo.

## Setting Up the Planning Problem

With the bridge classes in place, we can set up a complete planning problem. We will plan for a Panda arm navigating through a cage of sphere obstacles.

### Creating the Environment

First, define the obstacle environment. VAMP represents obstacles as geometric primitives (spheres, boxes, cylinders). Here we create a cage of 14 spheres:

~~~{.py}
import vamp
import numpy as np

# Sphere obstacle centers
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

radius = 0.2
env = vamp.Environment()
for sphere in problem:
    env.add_sphere(vamp.Sphere(sphere, radius))
~~~

### Configuring OMPL

Now we wire everything together using OMPL's ompl::geometric::SimpleSetup:

~~~{.py}
from ompl import base as ob
from ompl import geometric as og
from vamp_state_space import VampMotionValidator, VampStateValidityChecker, VampStateSpace

# Select the VAMP robot module
robot = vamp.panda
dimension = robot.dimension()

# Create the state space from the VAMP robot
space = VampStateSpace(robot=robot)
si = ob.SpaceInformation(space)

# Set VAMP-based validators
motion_validator = VampMotionValidator(si=si, env=env, robot=robot)
state_validity_checker = VampStateValidityChecker(si=si, env=env, robot=robot)
si.setMotionValidator(motion_validator)
si.setStateValidityChecker(state_validity_checker)

# Choose a planner
planner = og.RRTConnect(si)

# Build SimpleSetup
ss = og.SimpleSetup(si)
ss.setPlanner(planner)
~~~

### Setting Start and Goal States

Define start and goal configurations as joint angles for the 7-DOF Panda arm:

~~~{.py}
# Start configuration
a = [0., -0.785, 0., -2.356, 0., 1.571, 0.785]

# Goal configuration
b = [2.35, 1., 0., -0.8, 0, 2.5, 0.785]

start = si.allocState()
start[0:dimension] = a
goal = si.allocState()
goal[0:dimension] = b
ss.setStartAndGoalStates(start, goal)
~~~

### Solving

With everything configured, solving is a single call:

~~~{.py}
import time

planning_start = time.time()
result = ss.solve(5.0)  # 5 second time limit

if result:
    path = ss.getSolutionPath()
    print(f"Planning time: {time.time() - planning_start}")
    print(f"Generated path with {path.getStateCount()} states")
~~~

The planner will typically find a solution in milliseconds thanks to VAMP's fast collision checking.

## Benchmarking Planners

OMPL's ompl::tools::Benchmark class makes it easy to compare multiple planners on the same problem. Here we benchmark RRTConnect, RRT, KPIECE1, and LBKPIECE1:

~~~{.py}
from ompl import tools as ot

# Set up SimpleSetup as before (without setting a specific planner)
ss = og.SimpleSetup(si)
start = si.allocState()
start[0:dimension] = a
goal = si.allocState()
goal[0:dimension] = b
ss.setStartAndGoalStates(start, goal)

# Add planners to benchmark
planners = [
    og.RRTConnect(si),
    og.RRT(si),
    og.KPIECE1(si),
    og.LBKPIECE1(si),
]

benchmark = ot.Benchmark(ss, "Vamp Cage Planning Benchmark")
for planner in planners:
    benchmark.addPlanner(planner)

# Configure and run
req = ot.Request()
req.maxTime = 1.0       # 1 second per run
req.maxMem = 100.0      # 100 MB memory limit
req.runCount = 100       # 100 runs per planner
req.displayProgress = True
benchmark.benchmark(req)

# Save results
benchmark.saveResultsToFile("benchmark.log")
~~~

The log file can be visualized with [Planner Arena](http://plannerarena.org) (see the [benchmarking tutorial](benchmark.html) for details). You can also convert it to a SQLite database for custom analysis:

~~~{.py}
db_path = "benchmark.db"
ot.readBenchmarkLog(db_path, ["benchmark.log"], moveitformat=False)
~~~

## Running the Demo

The complete demo script supports both single planning and benchmarking via command-line flags:

~~~{.sh}
# Single planning run
python3 vamp/vamp_planning.py

# With 3D visualization (opens at localhost:8080)
python3 vamp/vamp_planning.py --visualize

# Benchmark multiple planners
python3 vamp/vamp_planning.py --benchmark --n_trials 100
~~~

## Extending to Other Robots

VAMP supports several robots out of the box. To plan for a different robot, simply change the robot module:

~~~{.py}
robot = vamp.ur5      # UR5 (6-DOF)
robot = vamp.fetch     # Fetch (8-DOF)
robot = vamp.baxter    # Baxter (7-DOF per arm)
~~~

The `VampStateSpace`, `VampStateValidityChecker`, and `VampMotionValidator` classes work with any VAMP robot module, since they read the dimension and bounds from the robot object.

For a more comprehensive example that loads problems from the [MotionBenchMaker](https://github.com/KavrakiLab/motion_bench_maker) dataset and supports multiple robots, see the `motion_benchmaker_demo.py` script in the demos directory.
