#!/usr/bin/env python3
"""
Core PyBullet visualization functionality for VAMP-OMPL
Handles robot loading, PyBullet initialization, and basic rendering
"""

import os
import pybullet as p
import pybullet_data
from typing import List, Optional


class PyBulletCore:
    """Core PyBullet functionality """
    
    def __init__(self, gui: bool = True, debug: bool = False):
        self.gui = gui
        self.debug = debug
        self.physics_client = None
        self.robot_id = None
        self.robot_name = None
        self.joint_indices = []
        
        self._initialize_pybullet()
    
    def _initialize_pybullet(self):
        """Initialize PyBullet with proper settings"""
        if self.gui:
            self.physics_client = p.connect(p.GUI)
            # Enable mouse interaction for camera control
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
            p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 1)
            p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 1)
        else:
            self.physics_client = p.connect(p.DIRECT)
        
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        # Load ground plane
        p.loadURDF("plane.urdf")
        
        # Set camera view
        p.resetDebugVisualizerCamera(
            cameraDistance=2.0,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=[0, 0, 0.5]
        )
        
        if self.debug:
            print(" PyBullet initialized with mouse interaction enabled")
    
    def load_robot(self, robot_name: str, urdf_path: str = "", 
                   base_position: List[float] = None, 
                   base_orientation: List[float] = None, 
                   use_fixed_base: bool = True, 
                   expected_joints: int = -1) -> bool:
        """Load robot with comprehensive configuration support"""
        
        # Set defaults
        if base_position is None:
            base_position = [0.0, 0.0, 0.0]
        if base_orientation is None:
            base_orientation = [0.0, 0.0, 0.0]
        
        # Handle URDF path resolution
        if not urdf_path:
            urdf_path = self._get_default_urdf_path(robot_name)
        
        resolved_urdf_path = self._find_urdf(urdf_path)
        
        # Convert Euler to quaternion
        base_orientation_quat = p.getQuaternionFromEuler(base_orientation)
        
        # Load robot
        self.robot_id = p.loadURDF(
            resolved_urdf_path, 
            base_position, 
            base_orientation_quat, 
            useFixedBase=use_fixed_base
        )
        
        if self.robot_id < 0:
            raise RuntimeError(f"Failed to load robot URDF: {resolved_urdf_path}")
        
        self.robot_name = robot_name
        
        # Get moveable joint indices
        self.joint_indices = self._get_moveable_joints()
        
        # Validate joint count
        if expected_joints > 0 and len(self.joint_indices) != expected_joints:
            print(f"  Warning: Expected {expected_joints} joints but found {len(self.joint_indices)} moveable joints")
        
        if self.debug:
            print(f" Loaded {robot_name} robot from {resolved_urdf_path}")
            print(f"  Moveable joints: {len(self.joint_indices)} (indices: {self.joint_indices})")
        
        return True
    
    def _get_default_urdf_path(self, robot_name: str) -> str:
        """Get default URDF path for built-in robots"""
        default_paths = {
            'panda': 'panda/panda_spherized.urdf',
            'ur5': 'ur5/ur5_spherized.urdf',
            'fetch': 'fetch/fetch_spherized.urdf'
        }
        
        if robot_name not in default_paths:
            raise ValueError(f"Unknown robot: {robot_name}. No URDF path provided and no default available.\n"
                           f"Built-in robots: {list(default_paths.keys())}.\n"
                           f"For custom robots, provide urdf_path in visualization configuration.")
        
        return default_paths[robot_name]
    
    def _find_urdf(self, urdf_path: str) -> str:
        """Find URDF file with smart path resolution"""
        
        # Try direct path first
        if os.path.exists(urdf_path):
            return urdf_path
        
        # Generate search paths based on path type
        if urdf_path.startswith("external/vamp/resources"):
            search_paths = [
                urdf_path,
                "../" + urdf_path,
                "../../" + urdf_path,
                "../../../" + urdf_path
            ]
        else:
            # Relative path within VAMP resources and config directories
            search_dirs = [
                "config",  # Local config directory
                "../demos/Vamp/config",  # From build directory
                "../../demos/Vamp/config",  # Deeper nesting
                "external/vamp/resources",
                "../external/vamp/resources",
                "../../external/vamp/resources", 
                "../../../external/vamp/resources"
            ]
            search_paths = [os.path.join(search_dir, urdf_path) for search_dir in search_dirs]
        
        # Try all paths
        for full_path in search_paths:
            if os.path.exists(full_path):
                return full_path
        
        # Error with detailed info
        error_msg = f"Could not find URDF: {urdf_path}\nSearched paths:\n"
        for path in search_paths:
            error_msg += f"  - {path} {'✓' if os.path.exists(path) else '✗'}\n"
        raise FileNotFoundError(error_msg)
    
    def _get_moveable_joints(self) -> List[int]:
        """Get indices of moveable (non-fixed) joints"""
        moveable_joints = []
        for i in range(p.getNumJoints(self.robot_id)):
            joint_info = p.getJointInfo(self.robot_id, i)
            if joint_info[2] != p.JOINT_FIXED:  # Not a fixed joint
                moveable_joints.append(i)
        return moveable_joints
    
    def set_joint_configuration(self, joint_angles: List[float]):
        """Set robot joint configuration"""
        if self.robot_id is None:
            raise RuntimeError("No robot loaded")
        
        if len(joint_angles) != len(self.joint_indices):
            raise ValueError(f"Expected {len(self.joint_indices)} joint angles, got {len(joint_angles)}")
        
        for joint_idx, angle in zip(self.joint_indices, joint_angles):
            p.resetJointState(self.robot_id, joint_idx, angle)
    
    def step_simulation(self):
        """Step the physics simulation"""
        p.stepSimulation()
    
    def cleanup(self):
        """Clean up PyBullet connection"""
        if self.physics_client is not None:
            p.disconnect()
            self.physics_client = None
            if self.debug:
                print(" PyBullet cleaned up") 