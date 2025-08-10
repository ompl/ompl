#!/usr/bin/env python3
"""
Environment and obstacle creation for VAMP-OMPL visualization
Handles creation of spheres, cuboids, capsules, and pointclouds in PyBullet
"""

import os
import pybullet as p
import numpy as np
from typing import List, Optional
from visualizer_config import ObstacleConfig, find_pointcloud_file


class EnvironmentBuilder:
    """Efficient environment and obstacle creation"""
    
    def __init__(self, debug: bool = False):
        self.debug = debug
        self.obstacle_ids = []
        self.pointcloud_ids = []
    
    def create_obstacles(self, obstacles: List[ObstacleConfig]) -> int:
        """Create all obstacles from configuration list"""
        
        self.obstacle_ids = []
        self.pointcloud_ids = []
        
        for obstacle in obstacles:
            try:
                obstacle_id = self._create_single_obstacle(obstacle)
                if obstacle_id is not None:
                    if obstacle.type == "pointcloud":
                        self.pointcloud_ids.extend(obstacle_id)  # Pointcloud returns list
                    else:
                        self.obstacle_ids.append(obstacle_id)
            except Exception as e:
                print(f"Warning: Failed to create obstacle '{obstacle.name}': {e}")
        
        total_obstacles = len(self.obstacle_ids) + len(self.pointcloud_ids)
        if self.debug:
            print(f" Created {total_obstacles} obstacles ({len(self.obstacle_ids)} primitives, {len(self.pointcloud_ids)} pointcloud spheres)")
        
        return total_obstacles
    
    def _create_single_obstacle(self, obstacle: ObstacleConfig):
        """Create a single obstacle based on its type"""
        
        if obstacle.type == "sphere":
            return self._create_sphere(obstacle)
        elif obstacle.type == "cuboid":
            return self._create_cuboid(obstacle)
        elif obstacle.type == "capsule":
            return self._create_capsule(obstacle)
        elif obstacle.type == "pointcloud":
            return self._create_pointcloud(obstacle)
        else:
            print(f"Warning: Unknown obstacle type '{obstacle.type}'. Skipping.")
            return None
    
    def _create_sphere(self, obstacle: ObstacleConfig) -> int:
        """Create sphere obstacle"""
        collision_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=obstacle.radius)
        visual_shape = p.createVisualShape(
            p.GEOM_SPHERE,
            radius=obstacle.radius,
            rgbaColor=[0.8, 0.1, 0.1, 0.7]  # Semi-transparent red
        )
        
        obstacle_id = p.createMultiBody(
            baseMass=0,  # Static obstacle
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=obstacle.position
        )
        
        if self.debug:
            print(f"  Created sphere '{obstacle.name}' at {obstacle.position}")
        
        return obstacle_id
    
    def _create_cuboid(self, obstacle: ObstacleConfig) -> int:
        """Create cuboid obstacle"""
        collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=obstacle.half_extents)
        visual_shape = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=obstacle.half_extents,
            rgbaColor=[0.1, 0.8, 0.1, 0.7]  # Semi-transparent green
        )
        
        # Convert Euler angles to quaternion
        orientation_quat = p.getQuaternionFromEuler(obstacle.orientation_euler_xyz)
        
        obstacle_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=obstacle.position,
            baseOrientation=orientation_quat
        )
        
        if self.debug:
            print(f" Created cuboid '{obstacle.name}' at {obstacle.position}")
        
        return obstacle_id
    
    def _create_capsule(self, obstacle: ObstacleConfig) -> int:
        """Create capsule obstacle"""
        collision_shape = p.createCollisionShape(
            p.GEOM_CAPSULE,
            radius=obstacle.radius,
            height=obstacle.length
        )
        visual_shape = p.createVisualShape(
            p.GEOM_CAPSULE,
            radius=obstacle.radius,
            length=obstacle.length,
            rgbaColor=[0.1, 0.1, 0.8, 0.7]  # Semi-transparent blue
        )
        
        # Convert Euler angles to quaternion
        orientation_quat = p.getQuaternionFromEuler(obstacle.orientation_euler_xyz)
        
        obstacle_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=obstacle.position,
            baseOrientation=orientation_quat
        )
        
        if self.debug:
            print(f" Created capsule '{obstacle.name}' at {obstacle.position}")
        
        return obstacle_id
    
    def _create_pointcloud(self, obstacle: ObstacleConfig) -> List[int]:
        """Create pointcloud obstacle as collection of small spheres"""
        try:
            # Find and load pointcloud file
            pointcloud_path = find_pointcloud_file(obstacle.pointcloud_file)
            points = self._load_pointcloud_file(pointcloud_path)
            
            if not points:
                print(f"Warning: No points loaded from {pointcloud_path}")
                return []
            
            # Create small spheres for each point
            sphere_ids = []
            collision_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=obstacle.point_radius)
            visual_shape = p.createVisualShape(
                p.GEOM_SPHERE,
                radius=obstacle.point_radius,
                rgbaColor=[0.8, 0.8, 0.1, 0.8]  # Semi-transparent yellow
            )
            
            for point in points:
                sphere_id = p.createMultiBody(
                    baseMass=0,
                    baseCollisionShapeIndex=collision_shape,
                    baseVisualShapeIndex=visual_shape,
                    basePosition=point
                )
                sphere_ids.append(sphere_id)
            
            if self.debug:
                print(f"  Created pointcloud '{obstacle.name}' with {len(points)} points from {pointcloud_path}")
            
            return sphere_ids
            
        except Exception as e:
            print(f"Warning: Failed to load pointcloud {obstacle.pointcloud_file}: {e}")
            return []
    
    def _load_pointcloud_file(self, filepath: str) -> List[List[float]]:
        """Load pointcloud from file (supports .xyz, .ply, .pcd)"""
        
        extension = os.path.splitext(filepath)[1].lower()
        
        if extension == '.xyz':
            return self._load_xyz(filepath)
        elif extension == '.ply':
            return self._load_ply(filepath)
        elif extension == '.pcd':
            return self._load_pcd(filepath)
        else:
            raise ValueError(f"Unsupported pointcloud format: {extension}")
    
    def _load_xyz(self, filepath: str) -> List[List[float]]:
        """Load XYZ format pointcloud"""
        points = []
        with open(filepath, 'r') as file:
            for line in file:
                line = line.strip()
                if line and not line.startswith('#'):
                    try:
                        coords = line.split()
                        if len(coords) >= 3:
                            point = [float(coords[0]), float(coords[1]), float(coords[2])]
                            points.append(point)
                    except ValueError:
                        continue
        return points
    
    def _load_ply(self, filepath: str) -> List[List[float]]:
        """Load PLY format pointcloud (ASCII only)"""
        points = []
        with open(filepath, 'r') as file:
            in_header = True
            vertex_count = 0
            
            for line in file:
                if in_header:
                    if line.startswith('element vertex'):
                        vertex_count = int(line.split()[2])
                    elif line.strip() == 'end_header':
                        in_header = False
                else:
                    try:
                        coords = line.split()
                        if len(coords) >= 3:
                            point = [float(coords[0]), float(coords[1]), float(coords[2])]
                            points.append(point)
                            if len(points) >= vertex_count:
                                break
                    except ValueError:
                        continue
        return points
    
    def _load_pcd(self, filepath: str) -> List[List[float]]:
        """Load PCD format pointcloud (ASCII only)"""
        points = []
        with open(filepath, 'r') as file:
            in_header = True
            
            for line in file:
                if in_header:
                    if line.startswith('DATA'):
                        in_header = False
                else:
                    try:
                        coords = line.split()
                        if len(coords) >= 3:
                            point = [float(coords[0]), float(coords[1]), float(coords[2])]
                            points.append(point)
                    except ValueError:
                        continue
        return points
    
    def cleanup(self):
        """Remove all created obstacles"""
        for obstacle_id in self.obstacle_ids + self.pointcloud_ids:
            try:
                p.removeBody(obstacle_id)
            except:
                pass  # Body might already be removed
        
        self.obstacle_ids = []
        self.pointcloud_ids = []
        
        if self.debug:
            print(" Environment cleaned up") 