#!/usr/bin/env python3
#
 # Software License Agreement (BSD License)
 #
 #  Copyright (c) 2015, Rice University
 #  All rights reserved.
 #
 #  Redistribution and use in source and binary forms, with or without
 #  modification, are permitted provided that the following conditions
 #  are met:
 #
 #   # Redistributions of source code must retain the above copyright
 #     notice, this list of conditions and the following disclaimer.
 #   # Redistributions in binary form must reproduce the above
 #     copyright notice, this list of conditions and the following
 #     disclaimer in the documentation and/or other materials provided
 #     with the distribution.
 #   # Neither the name of Rice University nor the names of its
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

 # Author: Sahruday Patti

"""
VAMP-OMPL Solution Visualizer 
Usage:
    python3 visualize_solution.py solution_path_file.txt [options]
"""

import argparse
import os
import sys
import time
from typing import Optional

# Import our modular components
from visualizer_core import PyBulletCore
from visualizer_config import ConfigParser, VisualizationConfig
from visualizer_environment import EnvironmentBuilder
from visualizer_animation import TrajectoryAnimator, InteractiveControls


class VampVisualizer:
    """Main VAMP visualization coordinator """
    
    def __init__(self, gui: bool = True, debug: bool = False):
        self.core = PyBulletCore(gui=gui, debug=debug)
        self.environment = EnvironmentBuilder(debug=debug)
        self.animator = TrajectoryAnimator(self.core, debug=debug)
        self.controls = InteractiveControls(self.animator)
        self.debug = debug
    
    def load_and_visualize(self, solution_file: str, yaml_config_file: Optional[str] = None,
                          duration: float = 10.0, loop: bool = False, 
                          draw_trajectory: bool = True) -> None:
        """Complete visualization workflow"""
        
        try:
            # Parse solution file
            waypoints, robot_name, env_name, viz_config = ConfigParser.parse_solution_file(solution_file)
            
            print(f" Loaded {len(waypoints)} waypoints for {robot_name}")
            if viz_config.urdf_path:
                print(" Using embedded visualization configuration")
            
            # Load robot
            self.core.load_robot(
                robot_name=robot_name,
                urdf_path=viz_config.urdf_path,
                base_position=viz_config.base_position,
                base_orientation=viz_config.base_orientation,
                use_fixed_base=viz_config.use_fixed_base,
                expected_joints=viz_config.expected_joints
            )
            
            # Load environment
            if yaml_config_file:
                print(f" Loading environment from {yaml_config_file}")
                obstacles = ConfigParser.load_yaml_config(yaml_config_file)
                self.environment.create_obstacles(obstacles)
            else:
                print(" Using empty environment (no YAML config provided)")
            
            # Set initial position
            self.core.set_joint_configuration(waypoints[0].tolist())
            
            # Enable interactive controls
            if self.core.gui:
                self.controls.enable_keyboard_controls()
            
            print(f"\n  Starting visualization...")
            print("  Use mouse to interact with camera:")
            print("   • Mouse wheel: Zoom in/out")
            print("   • Left-click + drag: Rotate camera")
            print("   • Right-click + drag: Pan camera")
            print("   • Ctrl-click + drag: Move camera target")
            
            # Start animation
            self.animator.animate_path(
                waypoints=waypoints,
                duration=duration,
                loop=loop,
                draw_trajectory=draw_trajectory
            )
            
            # Keep window open after animation
            if not loop and self.core.gui:
                print("\n Animation complete! Camera controls still active.")
                print("   Close window or press Ctrl+C to exit...")
                self._wait_for_exit()
            
        except Exception as e:
            print(f" Visualization error: {e}")
            raise
    
    def _wait_for_exit(self):
        """Wait for user to exit while maintaining interactive camera"""
        try:
            while True:
                # Keep PyBullet responsive for mouse interaction
                self.core.step_simulation()
                time.sleep(1.0 / 60.0)  # 60 FPS
        except KeyboardInterrupt:
            print("\n Visualization stopped by user")
    
    def cleanup(self):
        """Clean up all resources"""
        self.animator.cleanup()
        self.environment.cleanup()
        self.core.cleanup()


def main():
    """ main function with modular components"""
    parser = argparse.ArgumentParser(
        description="VAMP-OMPL Solution Visualizer - Modular & Interactive",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s solution_path.txt                     # Auto-configure from solution file
  %(prog)s solution_path.txt --yaml-config config.yaml  # Add environment
  %(prog)s solution_path.txt --duration 15 --loop       # 15s looping animation
  %(prog)s solution_path.txt --no-trajectory            # No trajectory line
        """
    )
    
    parser.add_argument("path_file", help="Solution path file")
    parser.add_argument("--yaml-config", help="YAML configuration file (optional)")
    parser.add_argument("--duration", type=float, default=10.0, help="Animation duration (seconds)")
    parser.add_argument("--loop", action="store_true", help="Loop animation")
    parser.add_argument("--no-trajectory", action="store_true", help="Don't draw trajectory line")
    parser.add_argument("--no-gui", action="store_true", help="Run without GUI")
    parser.add_argument("--debug", action="store_true", help="Enable debug output")
    
    args = parser.parse_args()
    
    if not os.path.exists(args.path_file):
        print(f" Solution file not found: {args.path_file}")
        return 1
    
    # Create visualizer
    visualizer = VampVisualizer(gui=not args.no_gui, debug=args.debug)
    
    try:
        # Run visualization
        visualizer.load_and_visualize(
            solution_file=args.path_file,
            yaml_config_file=args.yaml_config,
            duration=args.duration,
            loop=args.loop,
            draw_trajectory=not args.no_trajectory
        )
        
        return 0
        
    except KeyboardInterrupt:
        print("\n Visualization interrupted by user")
        return 0
    except Exception as e:
        print(f" Error: {e}")
        return 1
    finally:
        visualizer.cleanup()


if __name__ == "__main__":
    sys.exit(main()) 