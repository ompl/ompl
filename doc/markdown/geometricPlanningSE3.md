# Geometric Planning for a Rigid Body in 3D {#geometricPlanningSE3}

This tutorial shows how to plan for a rigid body in 3D. We show how to do this in two ways: [with](#geometricSimpleSetup) and [without](#withoutGeometricSimpleSetup) the ompl::geometric::SimpleSetup class. The main difference is that in the latter case ompl::base::SpaceInformation and ompl::base::ProblemDefinition need to be explicitly instantiated. Furthermore, the planner to be used must be explicitly instantiated as well. The recommended approach is using ompl::geometric::SimpleSetup as this is less bug prone and does not limit the functionality of the code in any way.

Setting up geometric planning for a rigid body in 3D requires the following steps:

- identify the space we are planning in: SE(3)
- select a corresponding state space from the available ones, or implement one. For SE(3), the ompl::base::SE3StateSpace is appropriate.
- since SE(3) contains an R<sup>3</sup> component, we need to define bounds.
- define the notion of state validity.
- define start states and a goal representation.

Once these steps are complete, the specification of the problem is conceptually done. The set of classes that allow the instantiation of this specification is shown below.

## Using the ompl::geometric::SimpleSetup Class {#geometricSimpleSetup}

\dontinclude RigidBodyPlanning.cpp
Assuming the following namespace definitions:
\skip ompl::base
\until ompl::geometric
And a state validity checking function defined like this:
\skipline isStateValid
We first create an instance of the state space we are planning in.
\skip planWithSimpleSetup
\until StateSpace
We then set the bounds for the R<sup>3</sup> component of this state space:
\skip RealVectorBounds
\until setBounds
Create an instance of ompl::geometric::SimpleSetup. Instances of ompl::base::SpaceInformation, and ompl::base::ProblemDefinition are created internally.
\skipline SimpleSetup
Set the state validity checker
\skipline setStateValidityChecker
Create a random start state:
\skip start(space)
\until start.random
And a random goal state:
\skip goal(space)
\until goal.random
Set these states as start and goal for SimpleSetup.
\skipline setStartAndGoalStates
We can now try to solve the problem. This will also trigger a call to ompl::geometric::SimpleSetup::setup() and create a default instance of a planner, since we have not specified one. Furthermore, ompl::base::Planner::setup() is called, which in turn calls ompl::base::SpaceInformation::setup(). This chain of calls will lead to computation of runtime parameters such as the state validity checking resolution. This call returns a value from ompl::base::PlannerStatus which describes whether a solution has been found within the specified amount of time (in seconds).  If this value can be cast to true, a solution was found.
\skipline ob::PlannerStatus solved
If a solution has been found, we can optionally simplify it and the display it
\skip solved
\until }

## Without ompl::geometric::SimpleSetup {#withoutGeometricSimpleSetup}

\dontinclude RigidBodyPlanning.cpp
Assuming the following namespace definitions:
\skip ompl::base
\until ompl::geometric
And a state validity checking function defined like this:
\skipline isStateValid
We first create an instance of the state space we are planning in.
\skip plan
\until StateSpace
We then set the bounds for the R<sup>3</sup> component of this state space:
\skip RealVectorBounds
\until setBounds
Create an instance of ompl::base::SpaceInformation for the state space
\skipline SpaceInformation
Set the state validity checker
\skipline setStateValidityChecker
Create a random start state:
\skip start(space)
\until start.random
And a random goal state:
\skip goal(space)
\until goal.random
Create an instance of ompl::base::ProblemDefinition
\skipline ProblemDefinition
Set the start and goal states for the problem definition.
\skipline setStartAndGoalStates
Create an instance of a planner
\skipline RRTConnect
Tell the planner which problem we are interested in solving
\skipline setProblemDefinition
Make sure all the settings for the space and planner are in order. This will also lead to the runtime computation of the state validity checking resolution.
\skipline planner->setup
We can now try to solve the problem. This call returns a value from ompl::base::PlannerStatus which describes whether a solution has been found within the specified amount of time (in seconds). If this value can be cast to true, a solution was found.
\skipline ob::PlannerStatus solved
If a solution has been found, we display it. Simplification could be done, but we would need to create an instance of ompl::geometric::PathSimplifier.
\skip solved
\until }
