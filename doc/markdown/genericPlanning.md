# Generic Instructions for Setting Up a Planning Context {#genericPlanning}

[TOC]

## Instantiating a state space

The first step for setting up a planning problem is selecting the space we are planning in. Simply create an instance of a class that inherits from ompl::base::StateSpace, or select one from the available implementations. To change the definition of the employed sampler or distance function, it is possible to provide a further specialization of such a state space. See [Implementing State Spaces]{implementingStateSpaces.html}.

~~~{.cpp}
base::StateSpacePtr space(new base::SE2StateSpace());
// set bounds ....
~~~

## Instantiating a control space

If planning with differential constraints, a representation of the space of controls is needed as well. Simply create an instance of a class that inherits from ompl::control::ControlSpace. Usually, ompl::control::RealVectorControlSpace is sufficient. An implementation needs to be specified for the ompl::control:ControlSpace::propagate() either by inheriting from the control space class or by calling ompl::control::ControlSpace::setPropagationFunction().

~~~{.cpp}
base::StateSpacePtr space(new base::SE2StateSpace());
// set bounds for state space
control::ControlSpacePtr cspace(new control::RealVectorControlSpace(space));
// set bounds for cspace
~~~

## Instantiating a space information class

Creating an actual instance of a space information class (ompl::base::SpaceInformation) is trivial as the constructor requires only a state space to be specified (ompl::base::StateSpace). When planning with controls (ompl::control::SpaceInformation), this constructor requires a control space (ompl::control::ControlSpace) as well.  The space information class also needs to be configured before use:

- ompl::base::StateValidityChecker is an abstract class that provides functionality for determining whether a state is valid or not. This class is assumed to be thread safe. The user should provide an implementation of this class and supply it to the space information instance by calling ompl::base::SpaceInformation::setStateValidityChecker(). Alternatively, the user can pass a function of the type ompl::base::StateValidityCheckerFn to ompl::base::SpaceInformation::setStateValidityChecker() instead. By default, all states are considered valid, if this parameter is not set.
- if using ompl::base::DiscreteMotionValidator for validating motions (this is the default), a call needs to be made to ompl::base::SpaceInformation::setStateValidityCheckingResolution() in order to specify the maximum distance between states to be checked for validity along a path segment. This distance is specified as a percentage of a space's maximum extent. If this call is not made, the resolution is assumed to be 1%. This value may be too low, in which case planning will be slower, or it may be too high, in which case it is possible to have collisions in solution plans.
\include svc.cpp
  Once the class is instantiated and parameters have been set, the ompl::base::SpaceInformation::setup() function needs to be called and the instance is ready for use.

## Instantiating a problem definition

- Instances of ompl::base::State or ompl::base::ScopedState need to be supplied as starting states for the system (at least one), using ompl::base::ProblemDefinition::addStartState().
- An ompl::base::Goal specification must be set using ompl::base::ProblemDefinition::setGoal(). For simplicity, specifications of this class are available: ompl::base::GoalRegion, ompl::base::GoalSampleableRegion, ompl::base::GoalState, ompl::base::GoalStates.
- As a simplification, a call can be made to ompl::base::ProblemDefinition::setStartAndGoalStates(). This will clear previous settings, add one start state and create an ompl::base::GoalState representation for the goal.
~~~{.cpp}
base::SpaceInformationPtr si(...);
base::ProblemDefinitionPtr pdef(new base::ProblemDefinition(si));

base::ScopedState start;
// fill start state

base::ScopedState goal;
// fill goal state

pdef->setStartAndGoalStates(start, goal);
~~~
  An important part of setting the problem definition is filling the content of start states (and goal states, if using a goal representation that requires states). Please see [Operating with states](workingWithStates.html#stateOps).

## Instantiating a planner

In order to use a motion planner (ompl::geometric::XXX, from __ompl/geometric/planners__ or ompl::control::XXX, from __ompl/control/planners__), an instance of ompl::base::SpaceInformation (ompl::control::SpaceInformation, respectively) must be available. This instance is supplied to the planner's constructor. After creation, a call to the planner's ompl::base::Planner::setup() method must be made and the planner instance is ready for use. The problem to be solved is set with ompl::base::Planner::setProblemDefinition(). The ompl::base::Planner::solve() method can be called repeatedly with different allowed time durations until a solution is found. The planning process continues with the available data structures when sequential calls to ompl::base::Planner::solve() are made. A call to ompl::base::Planner::clear() will restore a planner to its state before any calls to the ompl::base::Planner::solve() method were made.

~~~{.cpp}
using namespace ompl;
base::SpaceInformationPtr si(...);
base::ProblemDefinition   pdef(si);
// set start states & goal region for the problem definition

base::PlannerPtr planner(new geometric::SBL(si));
planner->setProblemDefinition(pdef);
planner->solve(1.0);
if (pdef->getGoal()->getSolutionPath())
{
   // do something with the solution
}
planner->clear();
~~~

### Using SimpleSetup

The instantiation of all the above classes is facilitated by the ompl::geometric::SimpleSetup class (ompl::control::SimpleSetup, respectively). Please see the documentation of these classes for more information. Making use of their functionality is recommended, if appropriate.
