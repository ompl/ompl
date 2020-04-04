# State Validity Checking {#stateValidation}

OMPL itself does not include code for state validity checking. This is intentional, since defining this notion depends on the type of problems to be solved. For instance, OMPL.app defines state validity checking in terms of collision checking between loaded CAD models. The ROS interface to OMPL in [MoveIt](https://moveit.ros.org) defines state validity checking to be collision checking between the robot model and sensed objects in the environment. If planning for protein folding, the state validity check has to do with the evaluation of energy functions.

In order to allow the user to specify the notion of state validity, OMPL defines two abstract classes:

- ompl::base::StateValidityChecker
- ompl::base::MotionValidator

These classes abstract away properties specific to the systems we are planning for. For instance, OMPL does not need to represent robot or obstacle geometry. The user can specify which implementations to use when configuring ompl::base::SpaceInformation. The instances of ompl::base::StateValidityChecker and ompl::base::MotionValidator need to be thread safe.

The ompl::base::StateValidityChecker class defines the ompl::base::StateValidityChecker::isValid() function, which allows planners to evaluate the validity of a state. If the user does not specify this class, an instance of ompl::base::AllValidStateValidityChecker is assumed. This will consider all states to be valid.

\include svc.cpp

The ompl::base::MotionValidator class defines ompl::base::MotionValidator::checkMotion() routines that evaluate the validity of motions between two specified states. By default, the implementation of this class is assumed to be ompl::base::DiscreteMotionValidator. The advantage of ompl::base::DiscreteMotionValidator is that it can be implemented solely using functionality from ompl::base::StateValidityChecker. The disadvantage is that the motion is discretized to some resolution and states are checked for validity at that resolution. If the resolution at which the motions are checked for validity is too large, there may be invalid states along the motion that escape undetected. If the resolution is too small, there can be too many states to be checked along each motion, slowing down the planner significantly. The resolution at which motions are discretized is computed using ompl::base::StateSpace::validSegmentCount(): each state space provides the ability to compute how many segments the motion should be split into so that the number of states checked for validity is satisfactory. The user can define this resolution in terms of percentages of the space's maximum extent (ompl::base::StateSpace::getMaximumExtent()) by calling ompl::base::SpaceInformation::setStateValidityCheckingResolution() or by calling ompl::base::StateSpace::setLongestValidSegmentFraction() for individual state spaces. Different resolutions can be used in subspaces, if using ompl::base::CompoundStateSpace. If continuous collision checking is available, it is recommended that a different implementation of ompl::base::MotionValidator is provided, one that does not rely on discretizing the motion at a specific resolution.

\include mv.cpp
