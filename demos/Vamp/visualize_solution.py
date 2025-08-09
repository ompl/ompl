#!/usr/bin/env python3
"""
VAMP-OMPL Solution Visualizer

YAML-driven obstacle visualization.

Usage:
    python3 visualize_solution.py --robot panda --yaml-config panda_demo.yaml path_file.txt
"""

import argparse
import os
import sys
import time
import numpy as np
import pybullet as p
import pybullet_data
from typing import List, Dict, Any, Optional, Tuple
import yaml
import math


def parse_visualization_config(solution_file_path: str) -> Dict[str, Any]:
    """Parse visualization configuration from solution file header"""
    viz_config = {}
    
    try:
        with open(solution_file_path, 'r') as file:
            for line in file:
                if not line.startswith('#'):
                    break  # End of header
                    
                # Parse visualization config lines
                if '# robot_name:' in line:
                    viz_config['robot_name'] = line.split(':', 1)[1].strip()
                elif '# urdf_path:' in line:
                    viz_config['urdf_path'] = line.split(':', 1)[1].strip()
                elif '# expected_joints:' in line:
                    viz_config['expected_joints'] = int(line.split(':', 1)[1].strip())
                elif '# base_position:' in line:
                    # Parse array format: [x, y, z]
                    pos_str = line.split(':', 1)[1].strip()
                    if pos_str.startswith('[') and pos_str.endswith(']'):
                        pos_values = pos_str[1:-1].split(',')
                        viz_config['base_position'] = [float(v.strip()) for v in pos_values]
                elif '# base_orientation:' in line:
                    # Parse array format: [rx, ry, rz]
                    orient_str = line.split(':', 1)[1].strip()
                    if orient_str.startswith('[') and orient_str.endswith(']'):
                        orient_values = orient_str[1:-1].split(',')
                        viz_config['base_orientation'] = [float(v.strip()) for v in orient_values]
                elif '# use_fixed_base:' in line:
                    viz_config['use_fixed_base'] = line.split(':', 1)[1].strip().lower() == 'true'
                    
    except Exception as e:
        print(f"Warning: Could not parse visualization config from {solution_file_path}: {e}")
    
    return viz_config


def find_pointcloud_file(pointcloud_filename: str) -> str:
    """Find pointcloud file in standard locations using the same strategy as C++"""
    search_paths = [
        pointcloud_filename,                               # Direct path
        f"demos/Vamp/{pointcloud_filename}",              # From project root
        f"../demos/Vamp/{pointcloud_filename}",           # From build dir
        f"../../demos/Vamp/{pointcloud_filename}",        # From deeper build dirs
        f"data/{pointcloud_filename}",                    # Local data dir
        f"config/{pointcloud_filename}"                   # Local config dir
    ]
    
    for candidate_path in search_paths:
        if os.path.exists(candidate_path):
            return candidate_path
    
    # If not found, raise an error with search paths
    error_message = f"Pointcloud file not found: {pointcloud_filename}\nSearched paths:\n"
    for search_path in search_paths:
        error_message += f"  - {search_path}\n"
    raise FileNotFoundError(error_message)


class ObstacleConfig:
    def __init__(self):
        self.type: str = ""
        self.name: str = ""
        self.position: List[float] = [0.0, 0.0, 0.0]
        self.orientation_euler_xyz: List[float] = [0.0, 0.0, 0.0]
        self.radius: float = 0.1
        self.half_extents: List[float] = [0.1, 0.1, 0.1]
        self.length: float = 0.2
        self.pointcloud_file: str = ""
        self.point_radius: float = 0.0025
        
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'ObstacleConfig':
        """Create ObstacleConfig from dictionary"""
        config = cls()
        config.type = data.get('type', '')
        config.name = data.get('name', '')
        config.position = data.get('position', [0.0, 0.0, 0.0])
        config.orientation_euler_xyz = data.get('orientation', data.get('orientation_euler_xyz', [0.0, 0.0, 0.0]))
        config.radius = data.get('radius', 0.1)
        config.half_extents = data.get('half_extents', [0.1, 0.1, 0.1])
        config.length = data.get('length', 0.2)
        config.pointcloud_file = data.get('pointcloud_file', '')
        config.point_radius = data.get('point_radius', 0.0025)
        
        # Validate obstacle type
        supported_types = ['sphere', 'cuboid', 'capsule', 'pointcloud']
        if config.type and config.type not in supported_types:
            print(f"  Warning: Unknown obstacle type '{config.type}'. Supported: {supported_types}")
        
        return config


class VampVisualizer:
    """Enhanced VAMP-OMPL solution path visualizer with data-driven obstacles"""
    
    def __init__(self, gui: bool = True, debug: bool = False):
        self.gui = gui
        self.debug = debug
        self.robot_id = None
        self.robot_name = None
        self.obstacle_ids = []
        self.joint_indices = []
        self.path_line_ids = []
        self.obstacle_configs = []  # Store obstacle configurations
        
        # Initialize PyBullet
        if self.gui:
            p.connect(p.GUI)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
        else:
            p.connect(p.DIRECT)
            
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        # Add ground plane
        p.loadURDF("plane.urdf")
        
        if self.debug:
            print("✓ PyBullet initialized")
            
    def load_robot(self, robot_name: str, urdf_path: str = "", base_position: List[float] = None, 
                   base_orientation: List[float] = None, use_fixed_base: bool = True, 
                   expected_joints: int = -1) -> str:
        """Load robot from URDF (now fully configurable)"""
        
        # If no URDF path provided, try to use defaults for built-in robots
        if not urdf_path:
            default_robot_paths = {
                'panda': 'panda/panda_spherized.urdf',
                'ur5': 'ur5/ur5_spherized.urdf', 
                'fetch': 'fetch/fetch_spherized.urdf'
            }
            if robot_name in default_robot_paths:
                urdf_path = default_robot_paths[robot_name]
                print(f"ℹ️  Using default URDF path for {robot_name}: {urdf_path}")
            else:
                raise ValueError(f"Unknown robot: {robot_name}. No URDF path provided and no default available.\n"
                               f"Built-in robots: {list(default_robot_paths.keys())}.\n"
                               f"For custom robots, provide urdf_path in visualization configuration.")
        
        # Resolve URDF path
        resolved_urdf_path = self._find_urdf(urdf_path)
        
        # Set default base position and orientation if not provided
        if base_position is None:
            base_position = [0.0, 0.0, 0.0]
        if base_orientation is None:
            base_orientation = [0.0, 0.0, 0.0]  # Euler angles
        
        # Convert Euler angles to quaternion for PyBullet
        base_orientation_quat = p.getQuaternionFromEuler(base_orientation)
        
        # Load robot
        self.robot_id = p.loadURDF(resolved_urdf_path, base_position, base_orientation_quat, useFixedBase=use_fixed_base)
        self.robot_name = robot_name
        
        # Get moveable joint indices
        self.joint_indices = []
        for i in range(p.getNumJoints(self.robot_id)):
            info = p.getJointInfo(self.robot_id, i)
            if info[2] != p.JOINT_FIXED:  # Not a fixed joint
                self.joint_indices.append(i)
        
        # Validate joint count if expected_joints specified
        if expected_joints > 0 and len(self.joint_indices) != expected_joints:
            print(f"⚠️  Warning: Expected {expected_joints} joints but found {len(self.joint_indices)} moveable joints")
            print(f"   Joint indices: {self.joint_indices}")
        
        print(f"✓ Loaded {robot_name} robot from {resolved_urdf_path}")
        print(f"  Moveable joints: {len(self.joint_indices)} (indices: {self.joint_indices})")
        return robot_name
        
    def _find_urdf(self, urdf_path: str) -> str:
        """Find URDF file in common locations (handles both relative and full paths)"""
        
        # First, try the path as-is (for absolute or relative paths from cwd)
        if os.path.exists(urdf_path):
            if self.debug:
                print(f"Found URDF at: {urdf_path}")
            return urdf_path
        
        # If the path already starts with external/vamp/resources, it's likely a full relative path
        if urdf_path.startswith("external/vamp/resources"):
            search_paths = [
                urdf_path,                           # Direct path
                "../" + urdf_path,                   # From build dir
                "../../" + urdf_path,                # From deeper build dirs
                "../../../" + urdf_path              # From even deeper build dirs
            ]
        else:
            # Treat as relative path within VAMP resources
            search_dirs = [
                "external/vamp/resources",
                "../external/vamp/resources", 
                "../../external/vamp/resources",
                "../../../external/vamp/resources"
            ]
            search_paths = [os.path.join(search_dir, urdf_path) for search_dir in search_dirs]
        
        # Try all search paths
        for full_path in search_paths:
            if os.path.exists(full_path):
                if self.debug:
                    print(f"Found URDF at: {full_path}")
                return full_path
                
        # Provide detailed error message with searched paths
        error_msg = f"Could not find URDF: {urdf_path}\nSearched paths:\n"
        for full_path in search_paths:
            error_msg += f"  - {full_path} {'✓' if os.path.exists(full_path) else '✗'}\n"
        error_msg += f"Make sure VAMP is properly installed and the external/vamp/resources directory exists."
        raise FileNotFoundError(error_msg)
        
    def create_environment_from_obstacles(self, obstacles: List[ObstacleConfig]):
        """Create environment from obstacle configurations (data-driven approach)"""
        self.obstacle_configs = obstacles
        self.obstacle_ids = []
        
        for obstacle in obstacles:
            obstacle_id = self._create_obstacle(obstacle)
            if obstacle_id is not None:
                self.obstacle_ids.append(obstacle_id)
                
        print(f"✓ Created environment with {len(self.obstacle_ids)} obstacles")
        
    def _create_obstacle(self, obstacle: ObstacleConfig) -> Optional[int]:
        """Create a single obstacle in PyBullet from configuration"""
        try:
            if obstacle.type == "sphere":
                return self._create_sphere_obstacle(obstacle)
            elif obstacle.type == "cuboid":
                return self._create_cuboid_obstacle(obstacle)
            elif obstacle.type == "capsule":
                return self._create_capsule_obstacle(obstacle)
            elif obstacle.type == "pointcloud":
                return self._create_pointcloud_obstacle(obstacle)
            else:
                print(f"Warning: Unknown obstacle type '{obstacle.type}'. Skipping obstacle.")
                return None
        except Exception as e:
            print(f"Error creating obstacle '{obstacle.name}': {e}")
            return None
            
    def _create_sphere_obstacle(self, obstacle: ObstacleConfig) -> int:
        """Create sphere obstacle"""
        collision_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=obstacle.radius)
        visual_shape = p.createVisualShape(
            p.GEOM_SPHERE, 
            radius=obstacle.radius,
            rgbaColor=[0.8, 0.1, 0.1, 0.7]
        )
        
        obstacle_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=obstacle.position
        )
        
        if self.debug:
            print(f"Created sphere '{obstacle.name}' at {obstacle.position} with radius {obstacle.radius}")
        return obstacle_id
        
    def _create_cuboid_obstacle(self, obstacle: ObstacleConfig) -> int:
        """Create cuboid (box) obstacle"""
        # Convert Euler angles to quaternion
        orientation_quat = p.getQuaternionFromEuler(obstacle.orientation_euler_xyz)
        
        collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=obstacle.half_extents)
        
        # Color coding for different cuboids
        color = [0.6, 0.4, 0.2, 1.0]  # Default brown
        if "table" in obstacle.name.lower():
            color = [0.6, 0.4, 0.2, 1.0]  # Brown for tables
        else:
            color = [0.1, 0.8, 0.1, 0.8]  # Green for other obstacles
            
        visual_shape = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=obstacle.half_extents,
            rgbaColor=color
        )
        
        obstacle_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=obstacle.position,
            baseOrientation=orientation_quat
        )
        
        if self.debug:
            print(f"Created cuboid '{obstacle.name}' at {obstacle.position} with half_extents {obstacle.half_extents}")
        return obstacle_id
        
    def _create_capsule_obstacle(self, obstacle: ObstacleConfig) -> int:
        """Create capsule obstacle"""
        # Convert Euler angles to quaternion
        orientation_quat = p.getQuaternionFromEuler(obstacle.orientation_euler_xyz)
        
        collision_shape = p.createCollisionShape(
            p.GEOM_CAPSULE, 
            radius=obstacle.radius,
            height=obstacle.length
        )
        visual_shape = p.createVisualShape(
            p.GEOM_CAPSULE,
            radius=obstacle.radius,
            length=obstacle.length,
            rgbaColor=[0.1, 0.1, 0.8, 0.8]  # Blue for capsules
        )
        
        obstacle_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=obstacle.position,
            baseOrientation=orientation_quat
        )
        
        if self.debug:
            print(f"Created capsule '{obstacle.name}' at {obstacle.position} with radius {obstacle.radius}, length {obstacle.length}")
        return obstacle_id
        
    def _create_pointcloud_obstacle(self, obstacle: ObstacleConfig) -> Optional[int]:
        """Create pointcloud obstacle by loading points and rendering as small spheres"""
        if not obstacle.pointcloud_file:
            print(f"Warning: No pointcloud file specified for obstacle '{obstacle.name}'")
            return None
            
        try:
            points = self._load_pointcloud_file(obstacle.pointcloud_file)
            if not points:
                print(f"Warning: No points loaded from '{obstacle.pointcloud_file}'")
                return None
                
            # Create compound visual shape for all points
            visual_shapes = []
            for point in points:
                visual_shapes.append(p.createVisualShape(
                    p.GEOM_SPHERE,
                    radius=obstacle.point_radius,
                    rgbaColor=[0.2, 0.9, 0.2, 0.9],  # Bright green for pointclouds
                    specularColor=[0.4, 0.4, 0.4]
                ))
            
            # Create collision shape (simplified as a single bounding box for performance)
            if len(points) > 0:
                points_array = np.array(points)
                min_bounds = np.min(points_array, axis=0)
                max_bounds = np.max(points_array, axis=0)
                center = (min_bounds + max_bounds) / 2
                half_extents = (max_bounds - min_bounds) / 2
                
                collision_shape = p.createCollisionShape(
                    p.GEOM_BOX,
                    halfExtents=half_extents.tolist()
                )
                
                # Create individual sphere bodies for visual representation
                point_ids = []
                for i, point in enumerate(points[:min(500, len(points))]):  # Limit for performance
                    point_id = p.createMultiBody(
                        baseMass=0,
                        baseCollisionShapeIndex=-1,  # No collision for individual points
                        baseVisualShapeIndex=visual_shapes[0],  # Reuse same visual shape
                        basePosition=point
                    )
                    point_ids.append(point_id)
                
                # Store point IDs for cleanup
                self.obstacle_ids.extend(point_ids)
                
                if self.debug:
                    print(f"Created pointcloud '{obstacle.name}' with {len(points)} points (showing {len(point_ids)})")
                
                return point_ids[0] if point_ids else None  # Return first point ID as representative
                
        except Exception as e:
            print(f"Error loading pointcloud '{obstacle.pointcloud_file}': {e}")
            return None
    
    def _load_pointcloud_file(self, filename: str) -> List[List[float]]:
        """Load pointcloud from file (supports .xyz, .ply, .pcd)"""
        try:
            # Resolve pointcloud file path using file locator
            resolved_path = find_pointcloud_file(filename)
            
            ext = os.path.splitext(resolved_path)[1].lower()
            
            if ext == '.xyz':
                return self._load_xyz_file(resolved_path)
            elif ext == '.ply':
                return self._load_ply_file(resolved_path)
            elif ext == '.pcd':
                return self._load_pcd_file(resolved_path)
            else:
                print(f"Unsupported pointcloud format: {ext}")
                return []
        except FileNotFoundError as e:
            print(f"Error locating pointcloud file: {e}")
            return []
    
    def _load_xyz_file(self, filename: str) -> List[List[float]]:
        """Load simple XYZ format (x y z per line)"""
        points = []
        with open(filename, 'r') as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) >= 3:
                    try:
                        points.append([float(parts[0]), float(parts[1]), float(parts[2])])
                    except ValueError:
                        continue
        return points
    
    def _load_ply_file(self, filename: str) -> List[List[float]]:
        """Load basic PLY format (ASCII only)"""
        points = []
        with open(filename, 'r') as f:
            in_header = True
            vertex_count = 0
            
            for line in f:
                if in_header:
                    if line.startswith('element vertex'):
                        vertex_count = int(line.split()[2])
                    elif line.strip() == 'end_header':
                        in_header = False
                else:
                    parts = line.strip().split()
                    if len(parts) >= 3:
                        try:
                            points.append([float(parts[0]), float(parts[1]), float(parts[2])])
                        except ValueError:
                            continue
                    if len(points) >= vertex_count:
                        break
        return points
    
    def _load_pcd_file(self, filename: str) -> List[List[float]]:
        """Load basic PCD format (ASCII only)"""
        points = []
        with open(filename, 'r') as f:
            in_header = True
            
            for line in f:
                if in_header:
                    if line.startswith('DATA'):
                        in_header = False
                else:
                    parts = line.strip().split()
                    if len(parts) >= 3:
                        try:
                            points.append([float(parts[0]), float(parts[1]), float(parts[2])])
                        except ValueError:
                            continue
        return points
        
    def create_environment(self, env_name: str):
        """Create environment - DEPRECATED: Use YAML configuration instead"""
        raise NotImplementedError(
            f"Named environments are no longer supported. Environment '{env_name}' must be "
            "defined explicitly via YAML configuration. Use --yaml-config option instead of --environment."
        )
        
    def load_yaml_config(self, yaml_file: str) -> List[ObstacleConfig]:
        """Load obstacle configurations from YAML file"""
        # Find YAML file in common locations
        search_paths = [
            yaml_file,
            f"config/{yaml_file}",
            f"demos/Vamp/config/{yaml_file}",
            f"../demos/Vamp/config/{yaml_file}",
            f"../../demos/Vamp/config/{yaml_file}"
        ]
        
        yaml_path = None
        for path in search_paths:
            if os.path.exists(path):
                yaml_path = path
                break
                
        if yaml_path is None:
            raise FileNotFoundError(f"Could not find YAML file: {yaml_file}")
            
        print(f" Loading YAML config: {yaml_path}")
        
        obstacles = []
        try:
            with open(yaml_path, 'r') as file:
                data = yaml.safe_load(file)
                
            # Extract obstacles if present
            if 'obstacles' in data and data['obstacles']:
                for obstacle_data in data['obstacles']:
                    obstacle = ObstacleConfig.from_dict(obstacle_data)
                    obstacles.append(obstacle)
                    
                print(f" Loaded {len(obstacles)} obstacles from YAML")
            else:
                print("No obstacles found in YAML, will use environment name if provided")
                
        except Exception as e:
            print(f"Error loading YAML config: {e}")
            
        return obstacles
        
    def set_joint_configuration(self, joint_values: np.ndarray):
        """Set robot joint positions"""
        if self.robot_id is None:
            return
            
        num_joints = min(len(joint_values), len(self.joint_indices))
        for i in range(num_joints):
            p.resetJointState(self.robot_id, self.joint_indices[i], joint_values[i])
            
    def get_end_effector_position(self) -> np.ndarray:
        """Get end-effector position"""
        if self.robot_id is None:
            return np.array([0, 0, 0])
            
        # Use last joint as end-effector
        ee_link = p.getNumJoints(self.robot_id) - 1 # Assuming the last joint is the end-effector
        link_state = p.getLinkState(self.robot_id, ee_link)
        return np.array(link_state[0])
        
    def animate_path(self, waypoints: np.ndarray, duration: float = 10.0, 
                    loop: bool = False, draw_trajectory: bool = True):
        """Animate robot motion along path"""
        if waypoints.shape[0] < 2:
            print("Need at least 2 waypoints for animation")
            return
            
        # Create start/goal markers
        self._create_markers(waypoints[0], waypoints[-1])
        
        print(f" Animating {waypoints.shape[0]} waypoints over {duration:.1f}s")
        
        dt = 1.0 / 60.0  # 60 FPS
        steps_per_waypoint = max(1, int(duration / (len(waypoints) * dt)))
        
        def run_animation():
            prev_ee_pos = None
            
            for i in range(len(waypoints) - 1):
                for step in range(steps_per_waypoint):
                    # Linear interpolation between waypoints
                    alpha = step / steps_per_waypoint
                    config = (1 - alpha) * waypoints[i] + alpha * waypoints[i + 1]
                    
                    self.set_joint_configuration(config)
                    
                    # Draw trajectory
                    if draw_trajectory:
                        ee_pos = self.get_end_effector_position()
                        if prev_ee_pos is not None and np.linalg.norm(ee_pos - prev_ee_pos) < 0.2:
                            line_id = p.addUserDebugLine(prev_ee_pos, ee_pos, 
                                                       lineColorRGB=[0.0, 0.5, 1.0], lineWidth=3.0)
                            self.path_line_ids.append(line_id)
                        prev_ee_pos = ee_pos.copy()
                    
                    p.stepSimulation()
                    time.sleep(dt)
                    
            # Final waypoint
            self.set_joint_configuration(waypoints[-1])
        
        try:
            if loop:
                print(" Looping animation (Ctrl+C to stop)")
                while True:
                    self._clear_trajectory()
                    run_animation()
            else:
                run_animation()
                print(f" Animation complete!")
        except KeyboardInterrupt:
            print("\nAnimation stopped")
            
    def _create_markers(self, start_config: np.ndarray, goal_config: np.ndarray):
        """Create start and goal markers"""
        # Get positions
        self.set_joint_configuration(start_config)
        start_pos = self.get_end_effector_position()
        
        self.set_joint_configuration(goal_config)
        goal_pos = self.get_end_effector_position()
        
        # Create markers
        p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=p.createVisualShape(p.GEOM_SPHERE, radius=0.04, 
                                                    rgbaColor=[0.0, 1.0, 0.0, 0.8]),
            basePosition=start_pos
        )
        p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=p.createVisualShape(p.GEOM_SPHERE, radius=0.04, 
                                                    rgbaColor=[1.0, 0.0, 0.0, 0.8]),
            basePosition=goal_pos
        )
        
        self.set_joint_configuration(start_config)
        print(f"Start: {start_pos}, Goal: {goal_pos}")
        
    def _clear_trajectory(self):
        """Clear trajectory lines"""
        for line_id in self.path_line_ids:
            try:
                p.removeUserDebugItem(line_id)
            except:
                pass
        self.path_line_ids.clear()
        
    def cleanup(self):
        """Clean up PyBullet resources"""
        try:
            p.disconnect()
        except:
            pass


def read_path_file(filepath: str) -> Tuple[np.ndarray, str, str, Dict[str, Any]]:
    """Read solution path file and extract waypoints, robot, environment, and visualization config"""
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"Path file not found: {filepath}")
        
    waypoints = []
    robot_name = None
    env_name = None
    
    # Parse visualization configuration from file header
    viz_config = parse_visualization_config(filepath)
    
    # Use robot name from visualization config if available
    if 'robot_name' in viz_config:
        robot_name = viz_config['robot_name']
        print(f"ℹ️  Robot name from visualization config: {robot_name}")
    
    # Fallback: Parse filename for robot/environment info (legacy support)
    if not robot_name:
        filename = os.path.basename(filepath)
        if filename.startswith("solution_path_"):
            parts = filename[14:-4].lower()  # Remove prefix and .txt
            
            # Legacy robot detection - now includes any robot name
            for robot in ["panda", "ur5", "fetch", "planar_arm_2dof", "articulated_arm_3dof"]:
                if robot in parts:
                    robot_name = robot
                    break
                    
            # Simple environment detection
            if "sphere" in parts:
                env_name = "sphere_cage"
            elif "table" in parts:
                env_name = "table_scene"
            elif "custom" in parts:
                env_name = "custom_environment"
            else:
                env_name = "empty"
    
    # Read waypoints
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith('#'):
                try:
                    joints = [float(x) for x in line.split()]
                    waypoints.append(joints)
                except ValueError:
                    continue
                    
    if not waypoints:
        raise ValueError(f"No valid waypoints found in {filepath}")
        
    return np.array(waypoints), robot_name or "unknown_robot", env_name or "empty", viz_config


def find_solution_files() -> List[str]:
    """Find solution path files"""
    search_dirs = [".", "demos/Vamp", "../demos/Vamp", "../../demos/Vamp"]
    files = []
    
    for search_dir in search_dirs:
        if os.path.exists(search_dir):
            dir_files = [f for f in os.listdir(search_dir) 
                        if f.startswith("solution_path_") and f.endswith(".txt")]
            files.extend([os.path.join(search_dir, f) for f in dir_files])
                
    return files


def interactive_mode() -> Optional[str]:
    """Interactive file selection"""
    files = find_solution_files()
    if not files:
        print(" No solution path files found!")
        return None
        
    print(f"\n Found {len(files)} solution files:")
    for i, f in enumerate(files):
        print(f"  {i+1}. {os.path.basename(f)}")
        
    try:
        choice = int(input(f"\nChoose file (1-{len(files)}): ")) - 1
        return files[choice]
    except (ValueError, IndexError):
        print(" Invalid choice")
        return None


def main():
    """Main function"""
    parser = argparse.ArgumentParser(
        description="Visualize VAMP-OMPL solution paths",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    
    parser.add_argument("path_file", nargs='?', help="Solution path file")
    parser.add_argument("--robot", help="Robot type (optional - auto-detected from path file if not specified)")
    parser.add_argument("--yaml-config", help="YAML configuration file (optional - uses embedded config if available)")
    parser.add_argument("--duration", type=float, default=10.0, help="Animation duration (seconds)")
    parser.add_argument("--loop", action="store_true", help="Loop animation")
    parser.add_argument("--no-trajectory", action="store_true", help="Don't draw trajectory")
    parser.add_argument("--no-gui", action="store_true", help="Run without GUI")
    parser.add_argument("--interactive", action="store_true", help="Interactive mode")
    
    args = parser.parse_args()
    
    # Interactive mode
    if args.interactive:
        print(" VAMP-OMPL Solution Path Visualizer")
        args.path_file = interactive_mode()
        if not args.path_file:
            return
    
    if not args.path_file:
        parser.print_help()
        return
        
    try:
        # Read path file with visualization configuration
        print(f" Reading: {args.path_file}")
        waypoints, detected_robot, detected_env, viz_config = read_path_file(args.path_file)
        print(f" Loaded {len(waypoints)} waypoints, {waypoints.shape[1]} joints")
        
        # Use specified robot or detected from path
        robot_name = args.robot or detected_robot
        if not robot_name:
            print("❌ Robot type could not be determined. Specify with --robot option or ensure path file contains robot info.")
            return
            
        print(f"🤖 Robot: {robot_name}")
        if viz_config:
            print(f"📁 Using embedded visualization configuration")
            
        # Initialize visualizer
        visualizer = VampVisualizer(gui=not args.no_gui, debug=True)
        
        # Load robot with visualization configuration
        visualizer.load_robot(
            robot_name, 
            urdf_path=viz_config.get('urdf_path', ''),
            base_position=viz_config.get('base_position', None),
            base_orientation=viz_config.get('base_orientation', None),
            use_fixed_base=viz_config.get('use_fixed_base', True),
            expected_joints=viz_config.get('expected_joints', -1)
        )
        
        # Load environment from YAML if provided, otherwise skip
        if args.yaml_config:
            print(f"🌍 Loading environment from YAML: {args.yaml_config}")
            obstacle_configs = visualizer.load_yaml_config(args.yaml_config)
            visualizer.create_environment_from_obstacles(obstacle_configs)
        else:
            print("ℹ️  No YAML config provided - creating empty environment")
            visualizer.create_environment_from_obstacles([])
        
        # Start animation
        visualizer.set_joint_configuration(waypoints[0])
        print(f"\n Press Enter to start animation...")
        input()
        
        visualizer.animate_path(waypoints, args.duration, 
                              args.loop, not args.no_trajectory)
        
        if not args.loop:
            print("\n Animation complete! Close window or press Ctrl+C to exit...")
            try:
                while True:
                    time.sleep(0.1)
            except KeyboardInterrupt:
                pass
        
    except Exception as e:
        print(f" Error: {e}")
    finally:
        if 'visualizer' in locals():
            visualizer.cleanup()


if __name__ == "__main__":
    main() 