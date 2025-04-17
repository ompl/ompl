import sys
import pytest

from ompl import base as ob
from ompl import geometric as og
from ompl import control as oc

import time
import random

class MyRRT(ob.Planner):
    def __init__(self, si, name):
        super().__init__(si, name)
        self._sampler = si.allocStateSampler()
        self._goal_bias = 0.05

    class TreeNode:
        def __init__(self, cur_state, parent_node):
            self.value = cur_state
            self.parent_node = parent_node
            self.children_nodes = []
            if parent_node is not None:
                parent_node.children_nodes.append(self)
    class Tree:
        def __init__(self, root_state, si):
            self.root = MyRRT.TreeNode(root_state, None)
            self.nodes = [self.root]
            self.si = si
        def add_state(self, cur_state, parent_node):
            new_node = MyRRT.TreeNode(cur_state, parent_node)
            self.nodes.append(new_node)
            return new_node
        def find_state(self, state, from_node = None):
            if from_node is None:
                from_node = self.root
            if from_node.value == state:
                return from_node
            for child in from_node.children_nodes:
                return self.find_state(state, child)
            return None
        def find_nearest(self, state):
            nearest_node = None
            nearest_dis = float('inf')
            for node in self.nodes:
                dis = self.si.distance(node.value, state)
                if dis < nearest_dis:
                    nearest_dis = dis
                    nearest_node = node
            return nearest_node

        def find_random(self):
            return random.choice(self.nodes)
        def backtrace(self, node):
            path = []
            while node:
                path.append(node.value)
                node = node.parent_node
            return path
        
    def solve(self, ptc):
        pdef = self.getProblemDefinition()
        goal = pdef.getGoal()
        can_sample = hasattr(goal, 'sampleGoal')
        if not can_sample:
            raise Exception("Unable to sample goal!")
        si = self.getSpaceInformation()
        pi = self.getPlannerInputStates()

        st = pi.nextStart()
        if not st:
            raise Exception("No start state!")
        # Initialize the tree
        tree = MyRRT.Tree(st, si)
        solution = None
        solved = False
        goal_node = None
        while not ptc():
            new_state = si.allocState()
            if random.random() < self._goal_bias:
                goal.sampleGoal(new_state)
            else:
                self._sampler.sampleUniform(new_state)
            nearest_node = tree.find_nearest(new_state)
            # print(nearest_node.value)
            if si.checkMotion(nearest_node.value, new_state):
                new_node = tree.add_state(new_state, nearest_node)
                dis_to_goal = goal.distanceGoal(new_state)
                if goal.isSatisfied(new_state):
                    solved = True
                    goal_node = new_node
                    break
        if solved:
            path = og.PathGeometric(si)
            path_states = tree.backtrace(goal_node)
            for s in path_states:
                path.append(s)
            pdef.addSolutionPath(path)
        return ob.PlannerStatus(solved, True)

def test_customize_planner_rvss():
    space = ob.RealVectorStateSpace(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(-1)
    bounds.setHigh(1)
    space.setBounds(bounds)

    def is_valid(state):
        return True
    
    si = ob.SpaceInformation(space)
    si.setStateValidityChecker(is_valid)
    si.setup()

    planner = MyRRT(si, "MyCustomPlanner")
    name = planner.getName()
    print("Planner name:", name)
    # 3) Create a ProblemDefinition with a start and goal state.
    start = si.allocState()
    start[0] = -0.5
    start[1] = -0.5
    goal = si.allocState()
    goal[0] = 0.5
    goal[1] = 0.5

    # 4) Create the RRT planner.
    # Let's choose addIntermediateStates=True for demonstration.
    rrt_planner = og.RRT(si, True)
    # 5) Configure some parameters
    rrt_planner.setGoalBias(0.1)
    rrt_planner.setRange(0.2)

    ss = og.SimpleSetup(si)
    ss.setStartAndGoalStates(start, goal)
    ss.setPlanner(rrt_planner)

    # Print them out
    print("Goal bias:", rrt_planner.getGoalBias())
    print("Range:", rrt_planner.getRange())
    print("Intermediate states:", rrt_planner.getIntermediateStates())


    # 7) Construct a PlannerTerminationCondition that stops after 1 second.
    # ptc = ob.PlannerTerminationCondition(lambda: True)

    start_time = time.time()
    # Create a termination condition that returns True after 5 seconds
    ptc = ob.PlannerTerminationCondition(
        lambda: (time.time() - start_time) > 5
    )

    # 8) Solve
    # result = ss.solve(1.0)
    result = ss.solve(ptc)
    print("Planner result:", result)

    # # 9) The `result` is a PlannerStatus object. Check if solution found:
    if result:
        print("Solution found!")
        # Optionally, retrieve the PathGeometric:
        solutionPath = ss.getSolutionPath()
        if solutionPath:
            print("Solution path length:", solutionPath.length())
            print("Solution path states:", solutionPath.getStateCount())
            print("Solution path:")
            solutionPath.printAsMatrix()
            print("Solution path:")
            solutionPath.print()
    else:
        print("No solution found within 1 second of planning time.")

def test_customize_planner_discrete():
    space = ob.DiscreteStateSpace(0,100_000)

    def is_valid(state):
        if state.value % 7 == 0:
            # print("State is valid:", state.value)
            return True
        else: 
            # print("State is invalid:", state.value)
            return False
    
    si = ob.SpaceInformation(space)
    si.setStateValidityChecker(is_valid)
    si.setup()

    planner = MyRRT(si, "MyCustomPlanner")
    name = planner.getName()
    print("Planner name:", name)
    # 3) Create a ProblemDefinition with a start and goal state.
    start = si.allocState()
    start.value = 7
    goal = si.allocState()
    goal.value = 7*123

    # 4) Create the RRT planner.
    rrt_planner = og.RRT(si, False)
    # 5) Configure some parameters
    rrt_planner.setGoalBias(0.1)
    rrt_planner.setRange(100)

    ss = og.SimpleSetup(si)
    ss.setStartAndGoalStates(start, goal)
    ss.setPlanner(rrt_planner)

    # Print them out
    print("Goal bias:", rrt_planner.getGoalBias())
    print("Range:", rrt_planner.getRange())
    print("Intermediate states:", rrt_planner.getIntermediateStates())


    # 7) Construct a PlannerTerminationCondition that stops after 1 second.
    ptc = ob.PlannerTerminationCondition(lambda: True)

    start_time = time.time()
    # Create a termination condition that returns True after 5 seconds
    ptc = ob.PlannerTerminationCondition(
        lambda: (time.time() - start_time) > 5
    )

    # 8) Solve
    # result = ss.solve(1.0)
    result = ss.solve(ptc)
    print("Planner result:", result)

    # # 9) The `result` is a PlannerStatus object. Check if solution found:
    if result:
        print("Solution found!")
        # Optionally, retrieve the PathGeometric:
        solutionPath = ss.getSolutionPath()
        if solutionPath:
            print("Solution path length:", solutionPath.length())
            print("Solution path states:", solutionPath.getStateCount())
            print("Solution path:")
            solutionPath.printAsMatrix()
            print("Solution path:")
            solutionPath.print()
            states = solutionPath.getStates()
            print(len(states))
            for state in states:
                print("State:", state.value)
                if state.value % 7 != 0:
                    print("State is invalid:", state.value)
                    break
    else:
        print("No solution found within 1 second of planning time.")

class MyControlRRT(ob.Planner):
    def __init__(self, si, name):
        super().__init__(si, name)
        self._sampler = si.allocStateSampler()
        self._goal_bias = 0.05

    class TreeNode:
        def __init__(self, cur_state:ob.State, cur_control:oc.Control, parent_node):
            self.value = cur_state
            self.parent_node = parent_node
            self.children_nodes = []
            if parent_node is not None:
                parent_node.children_nodes.append(self)
    class Tree:
        def __init__(self, root_state, si):
            self.root = MyRRT.TreeNode(root_state, None)
            self.nodes = [self.root]
            self.si = si
        def add_state(self, cur_state, parent_node):
            new_node = MyRRT.TreeNode(cur_state, parent_node)
            self.nodes.append(new_node)
            return new_node
        def find_state(self, state, from_node = None):
            if from_node is None:
                from_node = self.root
            if from_node.value == state:
                return from_node
            for child in from_node.children_nodes:
                return self.find_state(state, child)
            return None
        def find_nearest(self, state):
            nearest_node = None
            nearest_dis = float('inf')
            for node in self.nodes:
                dis = self.si.distance(node.value, state)
                if dis < nearest_dis:
                    nearest_dis = dis
                    nearest_node = node
            return nearest_node

        def find_random(self):
            return random.choice(self.nodes)
        def backtrace(self, node):
            path = []
            while node:
                path.append(node.value)
                node = node.parent_node
            return path
        
    def solve(self, ptc):
        pdef = self.getProblemDefinition()
        goal = pdef.getGoal()
        can_sample = hasattr(goal, 'sampleGoal')
        if not can_sample:
            raise Exception("Unable to sample goal!")
        si = self.getSpaceInformation()
        pi = self.getPlannerInputStates()
        sic = self.getControlSpace()
        st = pi.nextStart()
        if not st:
            raise Exception("No start state!")
        # Initialize the tree
        tree = MyRRT.Tree(st, si)
        solution = None
        solved = False
        goal_node = None
        while not ptc():
            new_state = si.allocState()
            if random.random() < self._goal_bias:
                goal.sampleGoal(new_state)
            else:
                self._sampler.sampleUniform(new_state)
            nearest_node = tree.find_nearest(new_state)
            # print(nearest_node.value)
            if si.checkMotion(nearest_node.value, new_state):
                new_node = tree.add_state(new_state, nearest_node)
                dis_to_goal = goal.distanceGoal(new_state)
                if goal.isSatisfied(new_state):
                    solved = True
                    goal_node = new_node
                    break
        if solved:
            path = og.PathGeometric(si)
            path_states = tree.backtrace(goal_node)
            for s in path_states:
                path.append(s)
            pdef.addSolutionPath(path)
        return ob.PlannerStatus(solved, True)
    def setup(*args, **argv):
        pass

def isStateValid(spaceInformation, state):
    # perform collision checking or check if other constraints are satisfied
    return spaceInformation.satisfiesBounds(state)

def propagate(temp1, control, duration, state):
    # For demonstration, intentionally messing up the partial usage
    # but let's keep it as your snippet.  Real usage would typically do cos/sin.
    state.setX(temp1.getX() + control[0] * duration * (temp1.getYaw()))
    state.setY(temp1.getY() + control[0] * duration * (temp1.getYaw()))
    state.setYaw(temp1.getYaw() + control[1] * duration)

def test_customize_control_planner():
    # 1) Construct the SE2 state space
    space = ob.SE2StateSpace()
    
    # set R^2 bounds
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(-1)
    bounds.setHigh(1)
    space.setBounds(bounds)

    # 2) Create a real-vector control space of dimension=2
    cspace = oc.RealVectorControlSpace(space, 2)
    cbounds = ob.RealVectorBounds(2)
    cbounds.setLow(-0.3)
    cbounds.setHigh(0.3)
    cspace.setBounds(cbounds)

    # 3) Construct a SpaceInformation from that (space, cspace)
    si = oc.SpaceInformation(space, cspace)
    si.setPropagationStepSize(1.0)

    # 4) Build a SimpleSetup from the SpaceInformation
    ss = oc.SimpleSetup(si)

    # 5) Provide a state validity checker as a lambda
    # This partial-lambda structure ensures the argument signature matches (State*) -> bool
    ss.setStateValidityChecker(lambda s: isStateValid(ss.getSpaceInformation(), s))
    
    # 6) Provide a state propagator
    ss.setStatePropagator(propagate)

    # 7) Create start and goal states
    start = space.allocState()
    start.setX(-0.5)
    start.setY(0.0)
    start.setYaw(0.0)

    goal = space.allocState()
    goal.setX(0.0)
    goal.setY(0.5)
    goal.setYaw(0.0)

    # 8) Set the start and goal states
    ss.setStartAndGoalStates(start, goal, 0.05)

    planner = MyControlRRT(si, "MyControlRRT")
    ss.setPlanner(planner)
    # 9) Attempt to solve
    solved = ss.solve(2)

    # If solved, optionally retrieve path
    if solved:
        print("Found solution path.")
        path = ss.getSolutionPath()
        path.printAsMatrix()

    del ss
    import gc
    gc.collect()

if __name__ == "__main__":
    # test_customize_planner_rvss()
    # test_customize_planner_discrete()
    test_customize_control_planner()