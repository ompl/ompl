######################################################################
# Software License Agreement (BSD License)
#
#  Copyright (c) 2026, Rice University
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Rice University nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
######################################################################

# Author: Jaewon Jung, Weihang Guo

import time
import numpy as np
import pyroki as pk
import viser
import os
import jax
import jax.numpy as jnp
import jax_dataclasses as jdc
import jaxlie
import jaxls
import json
from typing import Sequence
from pathlib import Path
from pyroki.collision import HalfSpace, RobotCollision, Sphere
from viser.extras import ViserUrdf

from ompl import base as ob
from ompl import geometric as og
from robot_descriptions.loaders.yourdfpy import load_robot_description

# ============================================================================
# Configuration
# ============================================================================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
VAMP_DIR = os.path.join(SCRIPT_DIR, "../../external/vamp")

DEFAULT_CONFIG = np.array([-np.pi/2, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0.0])
BASE_POS = np.array([0.0, 0.0, 0.0], dtype=float)

START_CONFIG: np.array = None
GOAL_CONFIG: np.array = None

# Store the current robot configuration as the global GOAL configuration
def _set_goal_config(current_config, goal_vis: ViserUrdf):
    global GOAL_CONFIG
    goal_vis.update_cfg(current_config)
    goal_vis.show_visual = True
    GOAL_CONFIG = current_config.copy()

# Store the current robot configuration as the global START configuration
def _set_start_config(current_config, start_vis: ViserUrdf):
    global START_CONFIG 
    start_vis.update_cfg(current_config)
    start_vis.show_visual = True
    START_CONFIG = current_config.copy()

def main():
    urdf = load_robot_description("ur5_description")
    target_link_name = "tool0"
    robot = pk.Robot.from_urdf(urdf)

    sphere_json_path = Path(__file__).parent.parent / "resources" / "ur5" / "ur5_spheres.json"
    print(sphere_json_path)
    with open(sphere_json_path, "r") as f:
        sphere_decomposition = json.load(f)

    # Create both collision models.
    robot_coll= RobotCollision.from_sphere_decomposition(
        sphere_decomposition=sphere_decomposition,
        urdf=urdf,
    )

    # Create a plane that ur5 stands upon
    plane_coll = HalfSpace.from_point_and_normal(
    np.array([0.0, 0.0, -0.15]), np.array([0.0, 0.0, 1.0])
    )   

    # Create a movable sphere obstacle
    sphere_obs = Sphere.from_center_and_radius(np.array([0.0, 0.0, 0.0]), 0.10)

    # Setting up a visualizer
    server = viser.ViserServer()
    server.scene.add_grid("/ground", width=2, height=2, cell_size=0.1)
    urdf_vis = ViserUrdf(server, urdf, root_node_name="/robot")

    # Create interactive controller for IK target.
    ik_target_handle = server.scene.add_transform_controls(
        "/ik_target", scale=0.2, position=(0.3, 0.0, 0.5), wxyz=(0, 0, 1, 0)
    )

    # Create interactive controller and mesh for the sphere obstacle.
    sphere_handle = server.scene.add_transform_controls(
        "/obstacle", scale=0.2, position=(0.4, 0.3, 0.4)
    )

    # Current configuration of the robot
    q_current = DEFAULT_CONFIG.copy()
    urdf_vis.update_cfg(q_current)

    goal_config_vis = ViserUrdf(server, urdf, root_node_name="/robot_goal", mesh_color_override=(0,255,0,0.5))
    goal_config_vis.show_visual = False
    set_goal_btn = server.gui.add_button("Set goal configuration")

    # Button logic for saving current q as goal config
    def on_goal_click(_):
        _set_goal_config(q_current, goal_config_vis)
    set_goal_btn.on_click(on_goal_click)

    start_config_vis = ViserUrdf(server, urdf, root_node_name="/robot_start", mesh_color_override=(255,0,0,0.5))
    start_config_vis.show_visual = False
    set_start_btn = server.gui.add_button("Set start configuration")

    # Button logic for saving current q as start config
    def on_start_click(_):
        _set_start_config(q_current, start_config_vis)
    set_start_btn.on_click(on_start_click)

    # State variables
    path = None
    path_i = 0
    is_playing = False


    def get_obstacles():
        obs_pos = np.array(sphere_handle.position, dtype=float)
        return {"sphere0": {"position": obs_pos, "radius": 0.10}}

    solve_btn = server.gui.add_button("Solve")

     # Button logic for solving
    def on_solve_click(_):
        nonlocal path, path_i, is_playing
        start = START_CONFIG if START_CONFIG is not None else q_current
        goal = GOAL_CONFIG
        
        if goal is None:
            print("Please set a goal configuration first using 'Set goal configuration'.")
            return
            
        print("Solving...")
        obs_dict = get_obstacles()
        
        solved, waypoints, msg = planner.plan(
            start_config=start,
            goal_config=goal,
            timeout=2.0,
            obstacles=obs_dict,
        )
        print(msg)
        
        if solved:
            path = waypoints
            path_i = 0
            is_playing = True # Auto-play heavily implied by "should execute the animation"

    solve_btn.on_click(on_solve_click)

    play_or_pause_btn = server.gui.add_button("Play/Pause")
    def on_play_or_pause_click(_):
        nonlocal is_playing
        if path is None:
            print("No path to play. Click 'Solve' first.")
            return
        is_playing = not is_playing
        print(f"Animation {'Resumed' if is_playing else 'Paused'}")
        
    play_or_pause_btn.on_click(on_play_or_pause_click)

    # Make obstacles visible to the user.
    server.scene.add_mesh_trimesh("/obstacle/mesh", mesh=sphere_obs.to_trimesh())

    # Shows the current status
    timing_handle = server.gui.add_number("Elapsed (ms)", 0.001, disabled=True)
    collision_handle = server.gui.add_checkbox("Robot in collision?", False, disabled=True)
    
    cc = CollisionChceker(robot_coll, plane_coll, robot)
    lower = np.array(robot.joints.lower_limits, dtype=float)
    upper = np.array(robot.joints.upper_limits, dtype=float)

    planner = MotionPlanner(cc,lower,upper, dim=6)

    # Main Loop
    while True:
        t0 = time.time()

        # 1) Update obstacles
        obs_dict = get_obstacles()
        cc.set_obs(obs_dict)
        
        # 2) Visualize collision status
        # Note: We check q_current collision
        collision_handle.value = (not cc.is_state_valid(q_current))

        # 3) Determine Robot State (Animation vs IK)
        if is_playing and path is not None:
             # Animation Mode
             if path_i < len(path):
                 q_current = path[path_i]
                 path_i += 1
             elif path_i == len(path):
                q_current = path[0]
                path_i = 1
             else:
                 is_playing = False # Stop at end
                 print("Animation Finished.")
        else:
             # Interactive / IK Mode
             # Allows user to drag the robot ghost to set start/goal
             try:
                world_coll_list = cc.global_obs_list()
                target_pos = np.array(ik_target_handle.position, dtype=float)
                target_wxyz = np.array(ik_target_handle.wxyz, dtype=float)
                
                # Check workspace validity (basic sphere check)
                if np.linalg.norm(target_pos - BASE_POS) < 1.0: # Rough workspace check
                    ik_result = PyrokiHelper.solve_ik_with_collision(
                        robot=robot,
                        coll=robot_coll,
                        world_coll_list=world_coll_list,
                        target_link_name=target_link_name, 
                        target_position=target_pos,
                        target_wxyz=target_wxyz,
                    )
                    q_current = np.array(ik_result, dtype=float)
             except Exception as e:
                # IK failed, keep current q_current
                pass
        
        # 4) Update Visualizer
        urdf_vis.update_cfg(q_current)

        # 5) Timing
        timing_handle.value = (time.time() - t0) * 1000.0
        time.sleep(0.01)

# ============================================================================
# CollisionChecker
# ============================================================================

class CollisionChceker():

    def __init__(self, robot_coll, plane_coll,robot):
        self.robot_coll = robot_coll
        self.plane_coll = plane_coll
        self.robot: pk.Robot = robot  
        self.obstacles = {}
    
    def set_obs(self, obstacle_dict :dict):
        self.obstacles = obstacle_dict or {}

    def global_obs_list(self):
        world = [self.plane_coll]

        for _, obs in self.obstacles.items():
            world.append(
                Sphere.from_center_and_radius(
                    np.asarray(obs["position"], dtype=np.float32),
                    float(obs["radius"]),
                )
            )
        return world


    def is_state_valid(self, q: np.ndarray) -> bool:
        """
        Check if a joint configuration is collision-free.
        
        Args:
            q: Joint configuration
        
        Returns:
            True if collision-free, False otherwise.
        """
        q = np.asarray(q, dtype=np.float32)
        world_list = self.global_obs_list()

        for world_coll in world_list:
            world_dist = self.robot_coll.compute_world_collision_distance(self.robot, q, world_coll)
            if float(jnp.min(world_dist)) <= -1e-2:
                print("world collision")
                return False
        
        self_dist = self.robot_coll.compute_self_collision_distance(self.robot, q)
        if float(jnp.min(self_dist)) <= -5e-2:
            print("self collision")
            return False
        return True 


# ============================================================================
# Motion Planner
# ============================================================================

class MotionPlanner():

    def __init__(self, collision_checker: CollisionChceker, lower_limits, upper_limits, dim):
        self.collision_checker = collision_checker
        self.dim = dim
        self.lower_limits = np.array(lower_limits, dtype=float)
        self.upper_limits = np.array(upper_limits, dtype=float)


    def plan(self, start_config: np.ndarray, goal_config: np.ndarray, 
            timeout: float, obstacles: dict = None) -> tuple:
        """
        Plan a path from start to goal using RRTConnect.
        
        Args:
            start_config: Start joint configuration
            goal_config: Goal joint configuration
            timeout: Planning timeout in seconds
            obstacles: Dict of obstacles - {obs_id: {'position': np.ndarray, 'radius': float}}
        """

        start_config = np.asarray(start_config, dtype=float).reshape(-1)
        goal_config = np.asarray(goal_config, dtype=float).reshape(-1)

        if start_config.shape[0] != self.dim or goal_config.shape[0] != self.dim:
            return False, None, f"Config must be shape ({self.dim},), got {start_config.shape}, {goal_config.shape}"
        
        self.collision_checker.set_obs(obstacles)
        
        space = ob.RealVectorStateSpace(self.dim)
        bounds = ob.RealVectorBounds(self.dim)
        for i in range(self.dim):
            bounds.setLow(i, float(self.lower_limits[i]))
            bounds.setHigh(i, float(self.upper_limits[i]))
        space.setBounds(bounds)
        
        ss = og.SimpleSetup(space)

        def _is_valid(state):
            q = np.array([state[i] for i in range(self.dim)], dtype=float)
            return bool(self.collision_checker.is_state_valid(q))

        ss.setStateValidityChecker(_is_valid)


        start = space.allocState()
        goal = space.allocState()
        for i in range(self.dim):
            start[i] = float(start_config[i])
            goal[i] = float(goal_config[i])
        
        si = ss.getSpaceInformation()
        ss.setStartAndGoalStates(start, goal)
        ss.setPlanner(og.RRTConnect(si))
        solved = ss.solve(timeout)
        
        if solved:
            ss.simplifySolution()
            path = ss.getSolutionPath()
            path.interpolate(int(50*path.length()))
            
            waypoints = []
            for i in range(path.getStateCount()):
                state = path.getState(i)
                waypoints.append(np.array([state[j] for j in range(self.dim)]))
            
            return True, waypoints, f"Path found with {len(waypoints)} waypoints!"
        
        return False, None, "No solution found within the time limit."
    
class PyrokiHelper(): 
    def __init__(self):
        pass
        
    def solve_ik_with_collision(
        robot: pk.Robot,
        coll: pk.collision.RobotCollision,
        world_coll_list: Sequence[pk.collision.CollGeom],
        target_link_name: str,
        target_position: np.ndarray,
        target_wxyz: np.ndarray,
    ) -> np.ndarray:
        """
        Solves the basic IK problem for a robot.

        Args:
            robot: PyRoKi Robot.
            target_link_name: Sequence[str]. Length: num_targets.
            position: ArrayLike. Shape: (num_targets, 3), or (3,).
            wxyz: ArrayLike. Shape: (num_targets, 4), or (4,).

        Returns:
            cfg: ArrayLike. Shape: (robot.joint.actuated_count,).
        """
        assert target_position.shape == (3,) and target_wxyz.shape == (4,)
        target_link_idx = robot.links.names.index(target_link_name)

        T_world_targets = jaxlie.SE3(
            jnp.concatenate([jnp.array(target_wxyz), jnp.array(target_position)], axis=-1)
        )
        cfg = PyrokiHelper.solve_ik_with_collision_jax(
            robot,
            coll,
            world_coll_list,
            T_world_targets,
            jnp.array(target_link_idx),
        )
        assert cfg.shape == (robot.joints.num_actuated_joints,)

        return np.array(cfg)


    @jdc.jit
    def solve_ik_with_collision_jax(
        robot: pk.Robot,
        coll: pk.collision.RobotCollision,
        world_coll_list: Sequence[pk.collision.CollGeom],
        T_world_target: jaxlie.SE3,
        target_link_index: jax.Array,
    ) -> jax.Array:
        """Solves the basic IK problem with collision avoidance. Returns joint configuration."""
        joint_var = robot.joint_var_cls(0)
        variables = [joint_var]

        # Weights and margins defined directly in factors
        costs = [
            pk.costs.pose_cost(
                robot,
                joint_var,
                target_pose=T_world_target,
                target_link_index=target_link_index,
                pos_weight=5.0,
                ori_weight=1.0,
            ),
            pk.costs.rest_cost(
                joint_var,
                rest_pose=jnp.array(joint_var.default_factory()),
                weight=0.01,
            ),
            pk.costs.self_collision_cost(
                robot,
                robot_coll=coll,
                joint_var=joint_var,
                margin=0.02,
                weight=5.0,
            ),
        ]
        costs.append(
            pk.costs.limit_constraint(
                robot,
                joint_var,
            )
        )
        costs.extend(
            [
                pk.costs.world_collision_constraint(
                    robot, coll, joint_var, world_coll, 0.05
                )
                for world_coll in world_coll_list
            ]
        )

        sol = (
            jaxls.LeastSquaresProblem(costs=costs, variables=variables)
            .analyze()
            .solve(verbose=False, linear_solver="dense_cholesky")
        )
        return sol[joint_var]

if __name__ == "__main__":
    main()