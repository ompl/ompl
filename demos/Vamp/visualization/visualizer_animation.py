#!/usr/bin/env python3
"""
Animation and trajectory rendering for VAMP-OMPL visualization
Handles robot motion and trajectory line visualization with efficient rendering
"""

import time
import numpy as np
import pybullet as p
from typing import List, Optional
from visualizer_core import PyBulletCore


class TrajectoryAnimator:
    """Efficient trajectory animation with smooth interpolation"""
    
    def __init__(self, pybullet_core: PyBulletCore, debug: bool = False):
        self.core = pybullet_core
        self.debug = debug
        self.trajectory_line_ids = []
        self.current_waypoint = 0
    
    def animate_path(self, waypoints: np.ndarray, duration: float = 10.0, 
                    loop: bool = False, draw_trajectory: bool = True, 
                    interpolation_steps: int = 10) -> None:
        """Animate robot following the path with smooth interpolation"""
        
        if waypoints.shape[0] < 2:
            print("Warning: Need at least 2 waypoints for animation")
            return
        
        # Draw trajectory line if requested
        if draw_trajectory:
            self._draw_trajectory_line(waypoints)
        
        print(f" Starting animation ({waypoints.shape[0]} waypoints, {duration:.1f}s)")
        print("   Mouse wheel: zoom, left-click+drag: rotate, right-click+drag: pan")
        print("   Press Ctrl+C to stop")
        
        try:
            self._run_animation_loop(waypoints, duration, loop, interpolation_steps)
        except KeyboardInterrupt:
            print("\n Animation stopped by user")
    
    def _draw_trajectory_line(self, waypoints: np.ndarray) -> None:
        """Draw trajectory as connected line segments using end-effector positions"""
        
        if self.core.robot_id is None:
            return
        
        # Clear previous trajectory lines
        self._clear_trajectory()
        
        # Get end-effector positions for all waypoints
        end_effector_positions = []
        
        for waypoint in waypoints:
            # Set robot to this configuration
            self.core.set_joint_configuration(waypoint.tolist())
            
            # Get end-effector position (use last link)
            num_joints = len(self.core.joint_indices)
            if num_joints > 0:
                link_state = p.getLinkState(self.core.robot_id, self.core.joint_indices[-1])
                end_effector_positions.append(link_state[0])  # World position
            else:
                # Fallback to base position
                base_pos, _ = p.getBasePositionAndOrientation(self.core.robot_id)
                end_effector_positions.append(base_pos)
        
        # Draw line segments between consecutive points
        if len(end_effector_positions) >= 2:
            for i in range(len(end_effector_positions) - 1):
                line_id = p.addUserDebugLine(
                    lineFromXYZ=end_effector_positions[i],
                    lineToXYZ=end_effector_positions[i + 1],
                    lineColorRGB=[0.0, 1.0, 0.0],  # Green trajectory
                    lineWidth=3.0
                )
                self.trajectory_line_ids.append(line_id)
        
        if self.debug:
            print(f" Drawn trajectory with {len(self.trajectory_line_ids)} line segments")
    
    def _run_animation_loop(self, waypoints: np.ndarray, duration: float, 
                           loop: bool, interpolation_steps: int) -> None:
        """Main animation loop with smooth interpolation"""
        
        start_time = time.time()
        total_waypoints = waypoints.shape[0]
        
        while True:
            current_time = time.time() - start_time
            
            if not loop and current_time >= duration:
                # Animation finished
                self.core.set_joint_configuration(waypoints[-1].tolist())
                print(f" Animation completed in {current_time:.1f}s")
                break
            
            # Calculate current position in animation
            if loop:
                # Loop animation
                progress = (current_time % duration) / duration
            else:
                # One-time animation
                progress = min(current_time / duration, 1.0)
            
            # Convert progress to waypoint index with interpolation
            waypoint_index_float = progress * (total_waypoints - 1)
            waypoint_index = int(waypoint_index_float)
            interpolation_factor = waypoint_index_float - waypoint_index
            
            # Get current joint configuration with interpolation
            if waypoint_index < total_waypoints - 1:
                # Interpolate between current and next waypoint
                current_config = self._interpolate_configurations(
                    waypoints[waypoint_index],
                    waypoints[waypoint_index + 1],
                    interpolation_factor
                )
            else:
                # Last waypoint
                current_config = waypoints[-1]
            
            # Update robot configuration
            self.core.set_joint_configuration(current_config.tolist())
            
            # Step simulation for visualization
            self.core.step_simulation()
            
            # Control animation speed (60 FPS target)
            time.sleep(1.0 / 60.0)
    
    def _interpolate_configurations(self, config1: np.ndarray, config2: np.ndarray, 
                                  factor: float) -> np.ndarray:
        """Smooth interpolation between two joint configurations"""
        
        # Linear interpolation for most joints
        interpolated = config1 + factor * (config2 - config1)
        
        # Handle angle wrapping for revolute joints (if needed)
        # more sophisticated angle interpolation
        # could be implemented for better results with large angle differences
        
        return interpolated
    
    def _clear_trajectory(self) -> None:
        """Clear all trajectory line segments"""
        for line_id in self.trajectory_line_ids:
            try:
                p.removeUserDebugItem(line_id)
            except:
                pass  # Line might already be removed
        
        self.trajectory_line_ids = []
    
    def goto_waypoint(self, waypoints: np.ndarray, waypoint_index: int) -> None:
        """Jump to specific waypoint (useful for interactive control)"""
        if 0 <= waypoint_index < waypoints.shape[0]:
            self.current_waypoint = waypoint_index
            self.core.set_joint_configuration(waypoints[waypoint_index].tolist())
            if self.debug:
                print(f"Jumped to waypoint {waypoint_index}/{waypoints.shape[0]-1}")
    
    def step_animation(self, waypoints: np.ndarray, forward: bool = True) -> None:
        """Step animation forward or backward by one waypoint"""
        if forward:
            if self.current_waypoint < waypoints.shape[0] - 1:
                self.current_waypoint += 1
        else:
            if self.current_waypoint > 0:
                self.current_waypoint -= 1
        
        self.core.set_joint_configuration(waypoints[self.current_waypoint].tolist())
        if self.debug:
            print(f"Stepped to waypoint {self.current_waypoint}/{waypoints.shape[0]-1}")
    
    def cleanup(self) -> None:
        """Clean up animation resources"""
        self._clear_trajectory()
        if self.debug:
            print(" Animation cleaned up")


class InteractiveControls:
    """Interactive controls for animation (keyboard/mouse)"""
    
    def __init__(self, animator: TrajectoryAnimator):
        self.animator = animator
        self.is_playing = False
        self.current_speed = 1.0
    
    def enable_keyboard_controls(self) -> None:
        """Enable keyboard controls for animation"""
        print("\n Interactive Controls:")
        print("  SPACE: Play/Pause")
        print("  ←/→:   Step backward/forward")
        print("  ↑/↓:   Speed up/slow down")
        print("  R:     Reset to start")
        print("  Mouse: Rotate/zoom/pan camera")
    
    def process_keyboard_input(self, waypoints: np.ndarray) -> bool:
        """Process keyboard input for interactive control"""
        
        keys = p.getKeyboardEvents()
        
        for key, state in keys.items():
            if state & p.KEY_WAS_TRIGGERED:
                if key == ord('r'):  # Reset
                    self.animator.goto_waypoint(waypoints, 0)
                    return True
                elif key == p.B3G_RIGHT_ARROW:  # Step forward
                    self.animator.step_animation(waypoints, forward=True)
                    return True
                elif key == p.B3G_LEFT_ARROW:  # Step backward
                    self.animator.step_animation(waypoints, forward=False)
                    return True
        
        return False 