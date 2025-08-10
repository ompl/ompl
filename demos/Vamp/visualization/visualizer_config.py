#!/usr/bin/env python3
"""
Configuration parsing and management for VAMP-OMPL visualization
Handles solution file parsing, YAML loading, and visualization config extraction
"""

import os
import yaml
import numpy as np
from typing import Dict, Any, List, Tuple


class VisualizationConfig:
    """Container for visualization configuration"""
    
    def __init__(self):
        self.robot_name: str = ""
        self.urdf_path: str = ""
        self.expected_joints: int = -1
        self.base_position: List[float] = [0.0, 0.0, 0.0]
        self.base_orientation: List[float] = [0.0, 0.0, 0.0]
        self.use_fixed_base: bool = True
        self.description: str = ""


class ObstacleConfig:
    """Container for obstacle configuration"""
    
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
        return config


class ConfigParser:
    """Handles parsing of solution files and YAML configurations"""
    
    @staticmethod
    def parse_solution_file(filepath: str) -> Tuple[np.ndarray, str, str, VisualizationConfig]:
        """Parse solution file and extract waypoints, robot, environment, and visualization config"""
        
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"Solution file not found: {filepath}")
        
        waypoints = []
        robot_name = ""
        env_name = ""
        viz_config = VisualizationConfig()
        
        # Parse header for visualization config
        with open(filepath, 'r') as file:
            for line in file:
                if not line.startswith('#'):
                    # Start of data section
                    break
                
                # Parse visualization config lines
                if '# robot_name:' in line:
                    viz_config.robot_name = line.split(':', 1)[1].strip()
                    robot_name = viz_config.robot_name
                elif '# urdf_path:' in line:
                    viz_config.urdf_path = line.split(':', 1)[1].strip()
                elif '# expected_joints:' in line:
                    viz_config.expected_joints = int(line.split(':', 1)[1].strip())
                elif '# base_position:' in line:
                    pos_str = line.split(':', 1)[1].strip()
                    if pos_str.startswith('[') and pos_str.endswith(']'):
                        pos_values = pos_str[1:-1].split(',')
                        viz_config.base_position = [float(v.strip()) for v in pos_values]
                elif '# base_orientation:' in line:
                    orient_str = line.split(':', 1)[1].strip()
                    if orient_str.startswith('[') and orient_str.endswith(']'):
                        orient_values = orient_str[1:-1].split(',')
                        viz_config.base_orientation = [float(v.strip()) for v in orient_values]
                elif '# use_fixed_base:' in line:
                    viz_config.use_fixed_base = line.split(':', 1)[1].strip().lower() == 'true'
        
        # Fallback robot detection from filename if not in header
        if not robot_name:
            robot_name = ConfigParser._detect_robot_from_filename(filepath)
        
        # Detect environment from filename  
        env_name = ConfigParser._detect_environment_from_filename(filepath)
        
        # Parse waypoints
        with open(filepath, 'r') as file:
            for line in file:
                line = line.strip()
                if line and not line.startswith('#'):
                    try:
                        joints = [float(x) for x in line.split()]
                        waypoints.append(joints)
                    except ValueError:
                        continue
        
        if not waypoints:
            raise ValueError(f"No valid waypoints found in {filepath}")
        
        return np.array(waypoints), robot_name, env_name, viz_config
    
    @staticmethod
    def _detect_robot_from_filename(filepath: str) -> str:
        """Detect robot type from filename """
        filename = os.path.basename(filepath).lower()
        
        # Check for known robot names
        robots = ["panda", "ur5", "fetch"]
        for robot in robots:
            if robot in filename:
                return robot
        
        return "unknown_robot"
    
    @staticmethod  
    def _detect_environment_from_filename(filepath: str) -> str:
        """Detect environment type from filename"""
        filename = os.path.basename(filepath).lower()
        
        if "sphere" in filename:
            return "sphere_cage"
        elif "custom" in filename:
            return "custom_environment"
        else:
            return "empty"
    
    @staticmethod
    def load_yaml_config(yaml_file: str) -> List[ObstacleConfig]:
        """Load obstacle configuration from YAML file"""
        
        if not os.path.exists(yaml_file):
            raise FileNotFoundError(f"YAML file not found: {yaml_file}")
        
        try:
            with open(yaml_file, 'r') as file:
                config = yaml.safe_load(file)
            
            obstacles = []
            if 'obstacles' in config:
                for obstacle_data in config['obstacles']:
                    try:
                        obstacle = ObstacleConfig.from_dict(obstacle_data)
                        obstacles.append(obstacle)
                    except Exception as e:
                        print(f"Warning: Failed to parse obstacle: {e}")
            
            return obstacles
            
        except yaml.YAMLError as e:
            raise ValueError(f"Failed to parse YAML file {yaml_file}: {e}")


def find_pointcloud_file(pointcloud_filename: str) -> str:
    """Find pointcloud file in standard locations """
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
    
    # If not found, raise error with search paths
    error_message = f"Pointcloud file not found: {pointcloud_filename}\nSearched paths:\n"
    for search_path in search_paths:
        error_message += f"  - {search_path}\n"
    raise FileNotFoundError(error_message) 