#!/usr/bin/env python3

"""
VAMP + OMPL Integration Demo

This demo showcases the integration of VAMP (Vector-Accelerated Motion Planning)
with OMPL for high-performance motion planning using Python bindings.

VAMP provides vectorized collision checking and motion validation that significantly
accelerates OMPL planners while maintaining the same interface and functionality.
"""

import sys
import time
import numpy as np
from enum import Enum
from typing import List, Tuple, Optional

# OMPL imports
import ompl.base as ob
import ompl.geometric as og
import ompl.util as ou

# VAMP imports
import vamp
import vamp.robots.panda as panda_robot
import vamp.robots.ur5 as ur5_robot
import vamp.robots.fetch as fetch_robot


class RobotType(Enum):
    PANDA = "panda"
    UR5 = "ur5"
    FETCH = "fetch"


class EnvironmentType(Enum):
    SPHERE_CAGE = "sphere_cage"
    TABLE_SCENE = "table_scene"
    EMPTY = "empty"


class PlannerType(Enum):
    BITSTAR = "bitstar"
    RRTCONNECT = "rrtconnect"
    PRM = "prm"


class VampOMPLIntegration:
    """VAMP + OMPL integration class for motion planning."""
    
    def __init__(self, robot_type: RobotType, environment_type: EnvironmentType, planner_type: PlannerType):
        self.robot_type = robot_type
        self.environment_type = environment_type
        self.planner_type = planner_type
        
        # Initialize robot based on type
        if robot_type == RobotType.PANDA:
            self.robot = panda_robot
        elif robot_type == RobotType.UR5:
            self.robot = ur5_robot
        elif robot_type == RobotType.FETCH:
            self.robot = fetch_robot
        else:
            raise ValueError(f"Unsupported robot type: {robot_type}")
        
        # Create environment
        self.environment = self._create_environment()
        
        # Setup OMPL components
        self._setup_ompl()
    
    def _create_environment(self):
        """Create the planning environment."""
        if self.environment_type == EnvironmentType.SPHERE_CAGE:
            return self._create_sphere_cage_environment()
        elif self.environment_type == EnvironmentType.TABLE_SCENE:
            return self._create_table_scene_environment()
        elif self.environment_type == EnvironmentType.EMPTY:
            return self._create_empty_environment()
        else:
            raise ValueError(f"Unsupported environment type: {self.environment_type}")
    
    def _create_sphere_cage_environment(self):
        """Create a sphere cage environment around the robot."""
        # Define sphere positions based on robot type
        if self.robot_type == RobotType.PANDA:
            sphere_positions = [
                [0.55, 0, 0.25], [0.35, 0.35, 0.25], [0, 0.55, 0.25],
                [-0.55, 0, 0.25], [-0.35, -0.35, 0.25], [0, -0.55, 0.25],
                [0.35, -0.35, 0.25], [0.35, 0.35, 0.8], [0, 0.55, 0.8],
                [-0.35, 0.35, 0.8], [-0.55, 0, 0.8], [-0.35, -0.35, 0.8],
                [0, -0.55, 0.8], [0.35, -0.35, 0.8]
            ]
        elif self.robot_type == RobotType.UR5:
            sphere_positions = [
                [0.4, 0, 0.3], [0.3, 0.3, 0.3], [0, 0.4, 0.3],
                [-0.4, 0, 0.3], [-0.3, -0.3, 0.3], [0, -0.4, 0.3],
                [0.3, -0.3, 0.3], [0.3, 0.3, 0.7], [0, 0.4, 0.7],
                [-0.3, 0.3, 0.7], [-0.4, 0, 0.7], [-0.3, -0.3, 0.7],
                [0, -0.4, 0.7], [0.3, -0.3, 0.7]
            ]
        else:  # FETCH
            sphere_positions = [
                [0.6, 0, 0.4], [0.4, 0.4, 0.4], [0, 0.6, 0.4],
                [-0.6, 0, 0.4], [-0.4, -0.4, 0.4], [0, -0.6, 0.4],
                [0.4, -0.4, 0.4], [0.4, 0.4, 1.0], [0, 0.6, 1.0],
                [-0.4, 0.4, 1.0], [-0.6, 0, 1.0], [-0.4, -0.4, 1.0],
                [0, -0.6, 1.0], [0.4, -0.4, 1.0]
            ]
        
        # Create spheres with radius 0.15
        radius = 0.15
        spheres = []
        for pos in sphere_positions:
            spheres.append(vamp.collision.factory.sphere.array(pos, radius))
        
        return spheres
    
    def _create_table_scene_environment(self):
        """Create a table scene environment with obstacles."""
        # Define box positions based on robot type
        if self.robot_type == RobotType.PANDA:
            box_positions = [
                # Table surface
                [0.0, 0.0, 0.0], [0.0, 0.0, 0.02],
                # Obstacles on table
                [0.3, 0.2, 0.1], [0.3, 0.2, 0.2],
                [-0.2, 0.3, 0.1], [-0.2, 0.3, 0.2],
                [0.1, -0.25, 0.1], [0.1, -0.25, 0.2]
            ]
        elif self.robot_type == RobotType.UR5:
            box_positions = [
                # Table surface
                [0.0, 0.0, 0.0], [0.0, 0.0, 0.02],
                # Obstacles on table
                [0.25, 0.15, 0.08], [0.25, 0.15, 0.16],
                [-0.15, 0.25, 0.08], [-0.15, 0.25, 0.16],
                [0.08, -0.2, 0.08], [0.08, -0.2, 0.16]
            ]
        else:  # FETCH
            box_positions = [
                # Table surface
                [0.0, 0.0, 0.0], [0.0, 0.0, 0.02],
                # Obstacles on table
                [0.4, 0.25, 0.12], [0.4, 0.25, 0.24],
                [-0.25, 0.4, 0.12], [-0.25, 0.4, 0.24],
                [0.15, -0.3, 0.12], [0.15, -0.3, 0.24]
            ]
        
        # Create boxes
        boxes = []
        for i in range(0, len(box_positions), 2):
            min_corner = box_positions[i]
            max_corner = box_positions[i + 1]
            boxes.append(vamp.collision.factory.box.array(min_corner, max_corner))
        
        return boxes
    
    def _create_empty_environment(self):
        """Create an empty environment."""
        return []
    
    def _get_start_goal_configurations(self):
        """Get robot-specific start and goal configurations."""
        if self.robot_type == RobotType.PANDA:
            start_config = [0., -0.785, 0., -2.356, 0., 1.571, 0.785]
            goal_config = [2.35, 1., 0., -0.8, 0, 2.5, 0.785]
        elif self.robot_type == RobotType.UR5:
            start_config = [0., -1.57, 0., -1.57, 0., 0.]
            goal_config = [1.57, -0.785, 0., -2.356, 0., 1.57]
        else:  # FETCH
            start_config = [0., 0., 0., 0., 0., 0., 0.]
            goal_config = [1.57, 0.785, 0., -1.57, 0., 0., 0.]
        
        return start_config, goal_config
    
    def _setup_ompl(self):
        """Setup OMPL state space, space information, and problem definition."""
        # Get robot dimension
        dimension = self.robot.dimension
        
        # Create state space
        space = ob.RealVectorStateSpace(dimension)
        
        # Set bounds (using normalized configuration space)
        bounds = ob.RealVectorBounds(dimension)
        for i in range(dimension):
            bounds.setLow(i, 0.0)
            bounds.setHigh(i, 1.0)
        space.setBounds(bounds)
        
        # Create space information
        self.si = ob.SpaceInformation(space)
        
        # Set state validity checker using VAMP
        self.si.setStateValidityChecker(ob.StateValidityCheckerFn(self._is_state_valid))
        
        # Set motion validator using VAMP
        self.si.setMotionValidator(ob.MotionValidatorFn(self._check_motion))
        
        self.si.setup()
        
        # Create problem definition
        self.pdef = ob.ProblemDefinition(self.si)
        
        # Set start and goal states
        start_config, goal_config = self._get_start_goal_configurations()
        
        start_state = ob.State(space)
        goal_state = ob.State(space)
        
        for i in range(dimension):
            start_state[i] = start_config[i]
            goal_state[i] = goal_config[i]
        
        self.pdef.setStartAndGoalStates(start_state, goal_state)
        
        # Set optimization objective
        obj = ob.PathLengthOptimizationObjective(self.si)
        self.pdef.setOptimizationObjective(obj)
    
    def _is_state_valid(self, state):
        """Check if a state is valid using VAMP."""
        # Convert OMPL state to VAMP configuration
        config = self._ompl_state_to_vamp_config(state)
        
        # Use VAMP to validate the configuration
        return self.robot.validate(config, self.environment)
    
    def _check_motion(self, s1, s2):
        """Check if motion between two states is valid using VAMP."""
        # Convert OMPL states to VAMP configurations
        config1 = self._ompl_state_to_vamp_config(s1)
        config2 = self._ompl_state_to_vamp_config(s2)
        
        # Use VAMP to validate the motion
        return self.robot.validate_motion(config1, config2, self.environment)
    
    def _ompl_state_to_vamp_config(self, state):
        """Convert OMPL state to VAMP configuration."""
        dimension = self.robot.dimension
        config_values = []
        
        for i in range(dimension):
            config_values.append(state[i])
        
        return np.array(config_values, dtype=np.float32)
    
    def _create_planner(self):
        """Create OMPL planner based on type."""
        if self.planner_type == PlannerType.BITSTAR:
            return og.BITstar(self.si)
        elif self.planner_type == PlannerType.RRTCONNECT:
            return og.RRTConnect(self.si)
        elif self.planner_type == PlannerType.PRM:
            return og.PRM(self.si)
        else:
            raise ValueError(f"Unsupported planner type: {self.planner_type}")
    
    def run_demo(self, planning_time=5.0, simplification_time=1.0, optimize=False):
        """Run the planning demo."""
        print(f"\n=== VAMP + OMPL Integration Demo ===")
        print(f"Robot: {self.robot_type.value}")
        print(f"Environment: {self.environment_type.value}")
        print(f"Planner: {self.planner_type.value}")
        print(f"Planning time: {planning_time}s")
        print(f"Optimize: {optimize}")
        
        # Create planner
        planner = self._create_planner()
        planner.setProblemDefinition(self.pdef)
        planner.setup()
        
        # Set optimization threshold if not optimizing
        if not optimize:
            obj = self.pdef.getOptimizationObjective()
            obj.setCostThreshold(obj.infiniteCost())
        
        # Solve the problem
        start_time = time.time()
        solved = planner.solve(planning_time)
        planning_duration = (time.time() - start_time) * 1000  # Convert to milliseconds
        
        print(f"\nPlanning completed in {planning_duration:.2f}ms")
        
        # Check solution
        if solved == ob.PlannerStatus.EXACT_SOLUTION:
            print("✓ Found solution! Simplifying path...")
            
            # Get solution path
            path = self.pdef.getSolutionPath()
            path_geometric = og.PathGeometric.cast(path)
            
            # Get initial cost
            obj = self.pdef.getOptimizationObjective()
            initial_cost = path_geometric.cost(obj).value()
            
            # Simplify the path
            simplifier = og.PathSimplifier(self.si, self.pdef.getGoal(), obj)
            simplify_start = time.time()
            simplified = simplifier.simplify(path_geometric, simplification_time)
            simplify_duration = (time.time() - simplify_start) * 1000
            
            if simplified:
                simplified_cost = path_geometric.cost(obj).value()
                print(f"✓ Path simplified in {simplify_duration:.2f}ms")
                print(f"Initial cost: {initial_cost:.4f}")
                print(f"Simplified cost: {simplified_cost:.4f}")
                print(f"Path length: {path_geometric.getStateCount()} states")
            else:
                print("✗ Path simplification failed")
            
            return True
        else:
            print("✗ No solution found")
            return False


def run_vamp_demo():
    """Run the main VAMP demo with multiple configurations."""
    print("VAMP + OMPL Integration Demo")
    print("============================")
    print("This demo showcases the integration of VAMP (Vector-Accelerated Motion Planning)")
    print("with OMPL for high-performance motion planning.")
    print()
    
    # Demo configurations
    demos = [
        (RobotType.PANDA, EnvironmentType.SPHERE_CAGE, PlannerType.BITSTAR, "Panda in Sphere Cage with BIT*"),
        (RobotType.PANDA, EnvironmentType.TABLE_SCENE, PlannerType.RRTCONNECT, "Panda in Table Scene with RRT-Connect"),
        (RobotType.UR5, EnvironmentType.SPHERE_CAGE, PlannerType.PRM, "UR5 in Sphere Cage with PRM"),
        (RobotType.FETCH, EnvironmentType.EMPTY, PlannerType.BITSTAR, "Fetch in Empty Environment with BIT*")
    ]
    
    success_count = 0
    total_count = 0
    
    for robot_type, env_type, planner_type, description in demos:
        print(f"\n{'-' * 60}")
        print(f"Demo: {description}")
        print(f"{'-' * 60}")
        
        try:
            # Create and run demo
            demo = VampOMPLIntegration(robot_type, env_type, planner_type)
            success = demo.run_demo(3.0, 0.5, False)
            
            if success:
                success_count += 1
            total_count += 1
            
        except Exception as e:
            print(f"✗ Error: {e}")
            total_count += 1
    
    print(f"\n{'=' * 60}")
    print("Demo Summary")
    print(f"{'=' * 60}")
    print(f"Successful demos: {success_count}/{total_count}")
    print(f"Success rate: {(100.0 * success_count / total_count):.1f}%")
    print()
    print("VAMP provides vectorized collision checking and motion validation")
    print("that significantly accelerates OMPL planners while maintaining")
    print("the same interface and functionality.")


if __name__ == "__main__":
    try:
        run_vamp_demo()
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1) 