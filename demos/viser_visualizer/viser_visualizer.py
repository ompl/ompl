"""Viser visualization class for VAMP robot trajectory visualization"""

import time
import threading
import sys
import termios
import tty
import select
import numpy as np
import vamp
import viser
from viser.extras import ViserUrdf
from robot_descriptions.loaders.yourdfpy import load_robot_description
from typing import Optional, List, Dict, Any
from scipy.spatial.transform import Rotation


class ViserVisualizer:
    
    def __init__(self, robot_name: str, port: Optional[int] = None):
        """Initialize the visualizer
        
        Args:
            robot_name: Name of the robot (must be available in vamp, e.g., 'panda', 'ur5', 'fetch')
            port: Optional port number for viser server (default: 8080)
        
        """
        self.robot_name = robot_name
        
        if not hasattr(vamp, robot_name):
            available_robots = [attr for attr in dir(vamp) if not attr.startswith('_')]
            raise ValueError(
                f"Robot '{robot_name}' not found in vamp. "
                f"Available robots: {available_robots}"
            )
        
        self.robot = getattr(vamp, robot_name)
        self.dimension = self.robot.dimension()
        
        if port is not None:
            self.server = viser.ViserServer(port=port)
        else:
            self.server = viser.ViserServer()
        
        # Load robot URDF from robot_descriptions
        description_name = f"{robot_name}_description"
        try:
            self.robot_urdf = load_robot_description(description_name)
        except Exception as e:
            raise ValueError(
                f"Could not load URDF for '{description_name}'. "
                f"Make sure robot_descriptions has this robot. Error: {e}"
            )
        
        self.urdf_vis = ViserUrdf(self.server, self.robot_urdf, root_node_name=f"/{robot_name}")
        
        self._trajectory = None
        self._slider = None
        self._playing = None
        
    def reset(self):
        """Reset the entire scene including the robot"""
        self.server.scene.reset()
        self.urdf_vis = ViserUrdf(self.server, self.robot_urdf, root_node_name=f"/{self.robot_name}")
        
        # Reset internal state
        self._trajectory = None
        self._slider = None
        self._playing = None
        self._start_time = None
    
    def load_mbm_environment(self, problem_data: Dict[str, Any], ignore_names: List[str] = [], 
                             color=(0.8, 0.4, 0.2), padding: float = 0.0):
        """Load environment from MBM problem format
        
        Args:
            problem_data: Dictionary containing 'sphere', 'cylinder', 'box' keys with obstacle data
            ignore_names: List of obstacle names to ignore
            color: RGB color tuple (0-1 range) for obstacles
            padding: Additional padding to add to obstacle sizes
        """
        # Helper function to convert euler angles to quaternion and create rotation matrix
        def euler_to_rotation_matrix(euler_xyz):
            """Convert euler angles (xyz) to rotation matrix"""
            return Rotation.from_euler('xyz', euler_xyz).as_matrix()
        
        # Load spheres
        for obj in problem_data.get("sphere", []):
            if obj['name'] not in ignore_names:
                position = np.array(obj["position"])
                radius = obj["radius"] + padding
                self.add_sphere(position=position, radius=radius, color=color, name=f"/sphere_{obj['name']}")
        
        # Used in vamp mbm evaluate, check evaluate_mbm.py on vamp/scripts
        is_box_problem = problem_data.get("problem") == "box"
        
        # Load cylinders (or as boxes if is_box_problem)
        for obj in problem_data.get("cylinder", []):
            if obj['name'] in ignore_names:
                continue
            
            position = np.array(obj["position"])
            orientation_euler = np.array(obj.get("orientation_euler_xyz", [0.0, 0.0, 0.0]))
            rotation_matrix = euler_to_rotation_matrix(orientation_euler)
            
            if is_box_problem:
                # Render cylinder as box (HACK for VAMP capsule overapproximation)
                radius = obj["radius"] + padding
                length = obj["length"]
                half_extents = [radius, radius, length / 2.0]
                self.add_box(position=position, half_extents=half_extents, 
                           rotation_matrix=rotation_matrix, color=color, name=f"/cylinder_as_box_{obj['name']}")
            else:
                radius = obj["radius"] + padding
                length = obj["length"]
                self.add_cylinder(position=position, radius=radius, length=length,
                                rotation_matrix=rotation_matrix, color=color, name=f"/cylinder_{obj['name']}")
        
        for obj in problem_data.get("box", []):
            if obj['name'] not in ignore_names:
                position = np.array(obj["position"])
                orientation_euler = np.array(obj.get("orientation_euler_xyz", [0.0, 0.0, 0.0]))
                rotation_matrix = euler_to_rotation_matrix(orientation_euler)
                half_extents = [h + padding/2 for h in obj["half_extents"]]
                self.add_box(position=position, half_extents=half_extents, 
                           rotation_matrix=rotation_matrix, color=color, name=f"/box_{obj['name']}")
    
    def add_sphere(self, position: np.ndarray, radius: float, color=(1, 0, 0), name: Optional[str] = None):
        """Add a sphere obstacle to the scene
        
        Args:
            position: 3D position [x, y, z]
            radius: Sphere radius
            color: RGB color tuple (0-1 range)
            name: Optional name for the sphere (auto-generated if not provided)
        """
        if name is None:
            name = f"/sphere_{len([k for k in self.server.scene._handles.keys() if k.startswith('/sphere_')])}"
        
        self.server.scene.add_icosphere(
            name=name,
            position=tuple(position),
            radius=radius,
            color=color
        )
    
    def add_box(self, position: np.ndarray, half_extents: List[float], 
                rotation_matrix: np.ndarray = None, color=(0.8, 0.4, 0.2), name: Optional[str] = None):
        """Add a box obstacle to the scene
        
        Args:
            position: 3D position [x, y, z]
            half_extents: Half extents [x, y, z] (full size will be 2x these values)
            rotation_matrix: 3x3 rotation matrix (identity if None)
            color: RGB color tuple (0-1 range)
            name: Optional name for the box (auto-generated if not provided)
        """
        if name is None:
            name = f"/box_{len([k for k in self.server.scene._handles.keys() if k.startswith('/box_')])}"
        
        # viser expects full extents, not half extents
        full_extents = [h * 2 for h in half_extents]
        
        if rotation_matrix is None:
            rotation_matrix = np.eye(3)
        
        rotation = Rotation.from_matrix(rotation_matrix)
        wxyz = rotation.as_quat()
        wxyz = np.array([wxyz[3], wxyz[0], wxyz[1], wxyz[2]])
        
        self.server.scene.add_box(
            name=name,
            dimensions=tuple(full_extents),
            position=tuple(position),
            wxyz=tuple(wxyz),
            color=color
        )
    
    def add_cylinder(self, position: np.ndarray, radius: float, length: float,
                    rotation_matrix: np.ndarray = None, color=(0.8, 0.4, 0.2), name: Optional[str] = None):
        """Add a cylinder obstacle to the scene
        
        Args:
            position: 3D position [x, y, z]
            radius: Cylinder radius
            length: Cylinder length (height)
            rotation_matrix: 3x3 rotation matrix (identity if None)
            color: RGB color tuple (0-1 range)
            name: Optional name for the cylinder (auto-generated if not provided)
        """
        if name is None:
            name = f"/cylinder_{len([k for k in self.server.scene._handles.keys() if k.startswith('/cylinder_')])}"
        
        if rotation_matrix is None:
            rotation_matrix = np.eye(3)
        
        rotation = Rotation.from_matrix(rotation_matrix)
        wxyz = rotation.as_quat()
        wxyz = np.array([wxyz[3], wxyz[0], wxyz[1], wxyz[2]])
        
        self.server.scene.add_cylinder(
            name=name,
            radius=radius,
            height=length,
            position=tuple(position),
            wxyz=tuple(wxyz),
            color=color
        )
    
    def add_grid(self, width: float = 2.0, height: float = 2.0, cell_size: float = 0.1):
        """Add a grid to the scene
        
        Args:
            width: Grid width
            height: Grid height
            cell_size: Size of each grid cell
        """
        self.server.scene.add_grid("/grid", width=width, height=height, cell_size=cell_size)
    
    def visualize_trajectory(self, trajectory: np.ndarray):
        """Visualize a robot trajectory with interactive controls
        
        Args:
            trajectory: Array of shape (timesteps, joints) containing joint configurations
        """
        if trajectory.ndim != 2:
            raise ValueError(f"Trajectory must be 2D array, got shape {trajectory.shape}")
        
        if trajectory.shape[1] != self.dimension:
            # Handle case where trajectory might have extra (gripper?) DOF
            if trajectory.shape[1] == self.dimension + 1:
                trajectory = trajectory[:, :self.dimension]
            else:
                raise ValueError(
                    f"Trajectory has {trajectory.shape[1]} DOFs but robot has {self.dimension} DOFs"
                )
        
        self._trajectory = trajectory
        
        self._slider = self.server.gui.add_slider(
            "Timestep", min=0, max=len(trajectory) - 1, step=1, initial_value=0
        )
        
        self._playing = self.server.gui.add_checkbox("Playing", initial_value=True)
        
        self._start_time = time.time()
    
    def visualization_step(self):
        """Perform one step of the visualization update"""
        if self._trajectory is None:
            return
        
        if self._playing.value:
            elapsed = time.time() - self._start_time
            progress = (elapsed % self._playtime) / self._playtime
            self._slider.value = int(progress * (len(self._trajectory) - 1))
        
        # Update robot configuration
        slider_idx = min(self._slider.value, len(self._trajectory) - 1)
        config = self._trajectory[slider_idx].tolist()
        
        # Some robots might need extra gripper DOF for visualization
        try:
            self.urdf_vis.update_cfg(config)
        except:
            config.append(1.5)
            self.urdf_vis.update_cfg(config)
        
        time.sleep(0.01)
    
    def visualization_loop(self):
        """Example visualization loop"""
        print(f"Visualization running on http://localhost:{self.server.port}")
        print("Press Ctrl+C to stop...")
        try:
            while True:
                self.visualization_step()
        except KeyboardInterrupt:
            print("\nVisualization stopped.")
    
    def play_once(self, dt =0.1):
        """
        Play through the trajectory once at the specified speed
        """
        if self._trajectory is None:
            print("No trajectory loaded. Call visualize_trajectory() first.")
            return
        
        for i in range(len(self._trajectory)):
            self._slider.value = i
            config = self._trajectory[i].tolist()
            try:
                self.urdf_vis.update_cfg(config)
            except:
                config.append(0.0)
                self.urdf_vis.update_cfg(config)
            time.sleep(dt)
            
    def play_until_key_pressed(self, key='any', dt=0.1):
        """Play visualization until specified key is pressed
        
        Args:
            key: Key to wait for. Use 'any' to stop on any key press (default: 'any')
            dt: Time delay between frames in seconds (default: 0.1)
            
        Returns:
            str: The key that was pressed
        """
        if self._trajectory is None:
            print("No trajectory loaded. Call visualize_trajectory() first.")
            return None
        
        print(f"Visualization running. Press {key if key != 'any' else 'any key'} to stop...")
        
        pressed_key = [None]
        stop_flag = threading.Event()
        
        def wait_for_key():
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(fd)
                while not stop_flag.is_set():
                    if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                        char = sys.stdin.read(1)
                        pressed_key[0] = char
                        if key == 'any' or char == key:
                            stop_flag.set()
                            break
                    time.sleep(0.01)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        
        # Start thread to wait for key
        key_thread = threading.Thread(target=wait_for_key, daemon=True)
        key_thread.start()
        
        # Run visualization loop until key is pressed
        while not stop_flag.is_set():
            for i in range(len(self._trajectory)):
                if stop_flag.is_set():
                    break
                self._slider.value = i
                config = self._trajectory[i].tolist()
                try:
                    self.urdf_vis.update_cfg(config)
                except:
                    config.append(0.0)
                    self.urdf_vis.update_cfg(config)
                time.sleep(dt)
        
        print(f"\nVisualization stopped. Key pressed: {pressed_key[0]}")
        return pressed_key[0]

