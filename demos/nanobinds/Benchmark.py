#!/usr/bin/env python

import sys
from os.path import abspath, dirname, join

try:
    from ompl import base as ob
    from ompl import geometric as og
    from ompl import tools as ot
except ImportError:
    # Assuming we are running from the build directory or standard install
    # If not, might need to adjust path
    try:
        from ompl import base as ob
        from ompl import geometric as og
        from ompl import tools as ot
    except ImportError:
        print("Could not import ompl. Please ensure it is installed.")
        sys.exit(1)

def myConfiguredPlanner(si):
    planner = og.RRTConnect(si)
    planner.setRange(100.0)
    return planner

def optionalPreRunEvent(planner):
    # do whatever configuration we want to the planner,
    # including changing of problem definition (input states)
    # via planner.getProblemDefinition()
    pass

def optionalPostRunEvent(planner, run):
    # do any cleanup, or set values for upcoming run (or upcoming call to the pre-run event).
    # adding elements to the set of collected run properties is also possible;
    # (the added data will be recorded in the log file)
    pass

def benchmark_example():
    # Create a state space for the space we are planning in
    space = ob.SE3StateSpace()
    bounds = ob.RealVectorBounds(3)
    bounds.setLow(-10)
    bounds.setHigh(10)
    space.setBounds(bounds)

    # Define a simple setup class
    ss = og.SimpleSetup(space)

    # Set state validity checking for this space
    class ValidityChecker(ob.StateValidityChecker):
        def isValid(self, state):
            return True

    ss.setStateValidityChecker(ValidityChecker(ss.getSpaceInformation()))

    # Define start and goal states
    start = space.allocState()
    # we can pick a random start state
    sampler = space.allocDefaultStateSampler()
    sampler.sampleUniform(start)
    
    goal = space.allocState()
    # and a random goal state
    sampler.sampleUniform(goal)
    
    ss.setStartAndGoalStates(start, goal)

    # First we create a benchmark class:
    b = ot.Benchmark(ss, "my experiment")

    # Optionally, specify some benchmark parameters (doesn't change how the benchmark is run)
    b.addExperimentParameter("num_dofs", "INTEGER", "6")
    b.addExperimentParameter("num_obstacles", "INTEGER", "10")

    # We add the planners to evaluate.
    b.addPlanner(og.KPIECE1(ss.getSpaceInformation()))
    b.addPlanner(og.RRT(ss.getSpaceInformation()))
    # b.addPlanner(og.SBL(ss.getSpaceInformation())) # SBL not bound
    b.addPlanner(og.LBKPIECE1(ss.getSpaceInformation()))

    # For planners that we want to configure in specific ways,
    # the PlannerAllocator should be used:
    b.addPlannerAllocator(myConfiguredPlanner)

    # After the Benchmark class is defined, the events can be optionally registered:
    b.setPreRunEvent(optionalPreRunEvent)
    b.setPostRunEvent(optionalPostRunEvent)

    # Now we can benchmark: 0.1 second time limit for each plan computation (short for demo),
    # 100 MB maximum memory usage per plan computation, 10 runs for each planner
    req = ot.Request()
    req.maxTime = 0.5 # s
    req.maxMem = 100.0 # MB
    req.runCount = 10
    req.displayProgress = True
    
    print("Starting benchmark...")
    b.benchmark(req)
    print("Benchmark done.")

    # This will generate a file of the form ompl_host_time.log
    filename = "ompl_benchmark_demo.log"
    b.saveResultsToFile(filename)
    print(f"Results saved to {filename}")

if __name__ == "__main__":
    benchmark_example()
