# Third-Party Contributions

This page includes a list of extensions to OMPL provided by the community. The contributed code can be found under the \c ompl/src/ompl/contrib directory in the OMPL source code. The contributed code has been written and tested by their respective authors listed below. Maintenance and support is also provided by the same authors. Please read the [directions on writing a contribution](contrib.html) and the suggested [style guide](styleGuide,html) if you would like to submit your own contribution.


# ROS Interface to OMPL

### Author: Sachin Chitta (Willow Garage)

- The [ompl_ros_interface](http://www.ros.org/wiki/ompl_ros_interface) ROS package wraps OMPL such that motion planners can be easily set up for your robot. Most configuration is automated. However, you will have to specify the parts of the robot for which you want to create a motion planner.


# RRT* and extensions

### Authors: Alejandro Perez and Sertac Karaman (MIT)

- [RRT*](\ref gRRTstar) (optimal [RRT](\ref gRRT)) is an asymptotically-optimal incremental sampling-based motion planning algorithm. RRT* algorithm is guaranteed to converge to an optimal solution, while its running time is guaranteed to be a constant factor of the running time of the RRT. The RRT* algorithm was introduced and analyzed in:

    - S. Karaman and E. Frazzoli, _Sampling-based algorithms for optimal motion planning_ Int. Journal of Robotics Research, 2011. Also available at http://arxiv.org/abs/1105.1186.

  For more information on RRT* and its variants see http://ares.lids.mit.edu/rrtstar.
- [Ball Tree RRT*](\ref gBallTreeRRTstar) is an implementation of RRT* that incorporates Ball Trees to approximate connected regions of free space with volumes in configuration space instead of points. Every vertex added to the tree has an initial volume of an infinite radius associated with it. This radius is gradually reduced as collisions are found. All samples within any of the existing volumes are discarded. However, discarded samples are collision checked. If a collision is found, the nearest volume is trimmed at the collision point. Information from all collision checking procedures within iterations is also used to trim volumes accordingly. The Ball Tree RRT* was introduced in:

    - A. Perez, S. Karaman, M. Walter, A. Shkolnik, E. Frazzoli, and S. Teller, _Asymptotically-optimal manipulation planning using incremental sampling-based algorithms,_ in IEEE/RSJ Int. Conf. on Intelligent Robots and Systems (IROS), 2011.


# PRM extensions

### Author: James Marble (University of Nevada at Reno)

- Generalized the implementation of PRM so that different variations can be created by passing connection strategies and filters. Provided such functions for PRM and PRM*.


# RRT extension for planning with controls

### Author: Jennifer Barry (MIT)

- Generalized the implementation of RRT (with controls) so that intermediate states generated along motions are also optionally added to that tree of motions.
