"""Viser helper class for robot trajectory visualization with OMPL"""

import time
import threading
import sys
import termios
import tty
import select
import numpy as np
from pathlib import Path
import viser
from viser.extras import ViserUrdf
from robot_descriptions.loaders.yourdfpy import load_robot_description
from typing import Optional, List, Dict, Any
from scipy.spatial.transform import Rotation


class ViserVisualizer:
    # Joint mappings for robots that use subset of URDF joints
    JOINT_MAPPINGS = {
        'fetch': [
            'torso_lift_joint',
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'upperarm_roll_joint',
            'elbow_flex_joint',
            'forearm_roll_joint',
            'wrist_flex_joint',
            'wrist_roll_joint',
            'head_pan_joint',
        ]
    }
    
    def __init__(self, robot_name: str, robot_dimension : int, port: Optional[int] = None):
        """Initialize the visualizer
        
        Args:
            robot_name: Name of the robot
            robot_dimension: Number of degrees of freedom for the robot
            port: Optional port number for viser server (default: 8080)
        
        """
        self.robot_name = robot_name
        self.dimension = robot_dimension
        
        # Setup joint mapping for this robot
        self.joint_mapping = self.JOINT_MAPPINGS.get(robot_name, None)
        
        if port is not None:
            self.server = viser.ViserServer(port=port)
        else:
            self.server = viser.ViserServer()
        
        # Load robot URDF from robot_descriptions
        description_name = f"{robot_name}_description"
        try:
            # VAMP uses ur5 with a Roboriq gripper, URDF is loaded from the repository
            if description_name == "ur5_description":
                import yourdfpy
                import vamp
                vamp_folder = Path(vamp.__file__).parent.parent.parent
                ur5_urdf_file = vamp_folder / 'resources' / 'ur5' / 'ur5.urdf'
                mesh_dir = vamp_folder / 'resources' / 'ur5'
                
                def package_aware_handler(fname):
                    """Resolve package:// URIs by stripping prefix and joining with mesh_dir"""
                    if fname.startswith('package://'):
                        fname = fname[len('package://'):]
                    resolved = Path(mesh_dir) / fname
                    return str(resolved)
                
                self.robot_urdf = yourdfpy.URDF.load(str(ur5_urdf_file), 
                                                     filename_handler=package_aware_handler,
                                                     mesh_dir=mesh_dir,
                                                     load_meshes=True, 
                                                     load_collision_meshes=True)
            else:
                # URDF loaded from robot_descriptions
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
    
    def _generate_name(self, prefix: str) -> str:
        """Generate a unique name based on existing objects with the same prefix
        
        Args:
            prefix: Name prefix (e.g., '/sphere_', '/box_', '/cylinder_')
            
        Returns:
            Unique name with numeric suffix
        """
        count = len([k for k in self.server.scene._handle_from_node_name.keys() if k.startswith(prefix)])
        return f"{prefix}{count}"
    
    def _rotation_to_wxyz(self, rotation_matrix: np.ndarray) -> np.ndarray:
        """Convert rotation matrix to wxyz quaternion format
        
        Args:
            rotation_matrix: 3x3 rotation matrix
            
        Returns:
            Quaternion in wxyz format
        """
        rotation = Rotation.from_matrix(rotation_matrix)
        quat = rotation.as_quat()  # Returns xyzw
        wxyz = np.array([quat[3], quat[0], quat[1], quat[2]])
        return wxyz
    
    def _update_robot_config(self, trajectory_idx: int, gripper_dof: float = 0.0) -> None:
        """Update the robot configuration in the visualization
        
        Args:
            trajectory_idx: Index into the current trajectory
            gripper_dof: Value for extra gripper DOF if needed (default: 0.0)
        """
        if self._trajectory is None:
            return
        
        idx = min(trajectory_idx, len(self._trajectory) - 1)
        plan_config = self._trajectory[idx].tolist()
        config = self._map_plan_config_to_urdf(plan_config)
        
        try:
            self.urdf_vis.update_cfg(config)
        except:
            config.append(gripper_dof)
            self.urdf_vis.update_cfg(config)
    
    def add_sphere(self, position: np.ndarray, radius: float, color=(1, 0, 0), name: Optional[str] = None):
        """Add a sphere obstacle to the scene
        
        Args:
            position: 3D position [x, y, z]
            radius: Sphere radius
            color: RGB color tuple (0-1 range)
            name: Optional name for the sphere (auto-generated if not provided)
        """
        if name is None:
            name = self._generate_name("/sphere_")
        
        self.server.scene.add_icosphere(
            name=name,
            position=tuple(position),
            radius=radius,
            color=color
        )
    
    def add_box(self, position: np.ndarray, half_extents: List[float], 
                rotation_matrix: Optional[np.ndarray] = None, color=(0.8, 0.4, 0.2), name: Optional[str] = None):
        """Add a box obstacle to the scene
        
        Args:
            position: 3D position [x, y, z]
            half_extents: Half extents [x, y, z] (full size will be 2x these values)
            rotation_matrix: 3x3 rotation matrix (identity if None)
            color: RGB color tuple (0-1 range)
            name: Optional name for the box (auto-generated if not provided)
        """
        if name is None:
            name = self._generate_name("/box_")
        
        # viser expects full extents, not half extents
        full_extents = [h * 2 for h in half_extents]
        
        if rotation_matrix is None:
            rotation_matrix = np.eye(3)
        
        wxyz = self._rotation_to_wxyz(rotation_matrix)
        
        self.server.scene.add_box(
            name=name,
            dimensions=tuple(full_extents),
            position=tuple(position),
            wxyz=tuple(wxyz),
            color=color
        )
    
    def add_cylinder(self, position: np.ndarray, radius: float, length: float,
                    rotation_matrix: Optional[np.ndarray] = None, color=(0.8, 0.4, 0.2), name: Optional[str] = None):
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
            name = self._generate_name("/cylinder_")
        
        if rotation_matrix is None:
            rotation_matrix = np.eye(3)
        
        wxyz = self._rotation_to_wxyz(rotation_matrix)
        
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
    
    def add_point_cloud(
        self,
        points: np.ndarray,
        color: Optional[np.ndarray] = None,
        point_size: float = 0.01,
    ):
        """Add a point cloud to the scene

        Args:
            points: Array of shape (N, 3) containing point coordinates
            color: Optional array of shape (N, 3) containing RGB colors for each point
        """
        if color is None:
            color = np.ones((points.shape[0], 3)) * np.array([1, 0, 0])

        self.server.scene.add_point_cloud(
            "/point_cloud", points=points, colors=color, point_size=point_size
        )

    def _map_plan_config_to_urdf(self, plan_config: List[float]) -> List[float]:
        """Map planning configuration to full URDF configuration
        
        For robots like fetch that use a subset of joints in planning, this method
        maps the planning DOFs to the correct positions in the full URDF configuration.
        
        Args:
            plan_config: Configuration from planner (e.g., 9 DOFs for fetch)
            
        Returns:
            Full URDF configuration with all joints
        """
        if self.joint_mapping is None:
            # No mapping needed, use config as-is
            return plan_config
        
        # Get all joint names from URDF
        all_joints = [joint.name for joint in self.robot_urdf.actuated_joints]
        n_total_joints = len(all_joints)
        
        # Create full configuration with zeros (neutral positions)
        full_config = [0.0] * n_total_joints
        
        # Map planning joints to their positions in URDF
        for planning_idx, joint_name in enumerate(self.joint_mapping):
            if joint_name in all_joints:
                urdf_idx = all_joints.index(joint_name)
                if planning_idx < len(plan_config):
                    full_config[urdf_idx] = plan_config[planning_idx]
        
        return full_config
    
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
        
        # Update robot configuration with consistent gripper DOF of 0.0
        slider_idx = self._slider.value
        self._update_robot_config(slider_idx, gripper_dof=0.0)
        
        time.sleep(0.01)
    
    def visualization_loop(self):
        """Visualization loop"""
        print(f"Visualization running on http://localhost:{self.server.port}")
        print("Press Ctrl+C to stop...")
        try:
            while True:
                self.visualization_step()
        except KeyboardInterrupt:
            print("\nVisualization stopped.")
    
    def play_once(self, dt=0.1):
        """
        Play through the trajectory once at the specified speed
        
        Args:
            dt: Time delay between frames in seconds (default: 0.1)
        """
        if self._trajectory is None:
            print("No trajectory loaded. Call visualize_trajectory() first.")
            return
        
        for i in range(len(self._trajectory)):
            self._slider.value = i
            self._update_robot_config(i, gripper_dof=0.0)
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
            try:
                fd = sys.stdin.fileno()
                old_settings = termios.tcgetattr(fd)
            except:
                # Not a TTY, skip key handling
                return
            
            try:
                tty.setraw(fd)
                while not stop_flag.is_set():
                    try:
                        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                            char = sys.stdin.read(1)
                            pressed_key[0] = char
                            if key == 'any' or char == key:
                                stop_flag.set()
                                break
                    except:
                        break
                    time.sleep(0.01)
            except Exception as e:
                print(f"Key listener error: {e}")
            finally:
                try:
                    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                except:
                    pass
        
        # Start thread to wait for key
        key_thread = threading.Thread(target=wait_for_key, daemon=True)
        key_thread.start()
        
        # Run visualization loop until key is pressed
        try:
            while not stop_flag.is_set():
                for i in range(len(self._trajectory)):
                    if stop_flag.is_set():
                        break
                    self._slider.value = i
                    self._update_robot_config(i, gripper_dof=0.0)
                    time.sleep(dt)
        finally:
            stop_flag.set()
        
        print(f"\nVisualization stopped. Key pressed: {pressed_key[0]}")
        return pressed_key[0]

