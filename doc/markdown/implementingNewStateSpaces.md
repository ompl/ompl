# Implementing State Spaces {#implementingStateSpaces}

## Combining existing state spaces

The simplest way to obtain new state spaces is to combine existing ones. For example, to get the state space of a manipulator arm one could combine R<sup>5</sup> (ompl::base::RealVectorStateSpace) and SO2 (ompl::base::SO2StateSpace) to represent 5 joints that have bounds and one joint that can rotate continuously:

~~~{.cpp}
ompl::base::StateSpacePtr r5(new ompl::base::RealVectorStateSpace(5));
ompl::base::StateSpacePtr so2(new ompl::base::SO2StateSpace())
ompl::base::StateSpacePtr newSpace = r5 + so2;
~~~

Alternatively to using the “+” operator on state spaces (see [working with states and state spaces](workingWithStates.html)), one could directly create an instance of ompl::base::CompoundStateSpace and call ompl::base::CompoundStateSpace::addSubspace() on it. This approach allows setting the weights of each added subspace for computing distances bewteen states in the compound state space. When using the “+” operator these weights are assumed to be 1.0.

~~~{.cpp}
ompl::base::CompoundStateSpace *newSpace = new ompl::base::CompoundStateSpace();
newSpace->addSubspace(ompl::base::StateSpacePtr(new ompl::base::RealVectorStateSpace(5)), 1.0);
newSpace->addSubspace(ompl::base::StateSpacePtr(new ompl::base::SO2StateSpace()), 0.5);
~~~

## Inheriting from existing state spaces

In order to implement a new state space it is necessary to define a class that inherits from an existing state space class (either the ompl::base::StateSpace class or an existing state space implementation). All state space specific functions (pure virtual in the ompl::base::StateSpace class) need to be implemented accordingly. If the implementation of the new state space uses a new state type (even if it inherits from an existing state type), that state type must be named __StateType__. To use the newly defined state type the ompl::base::StateSpace::allocState() and ompl::base::StateSpace::freeState() functions must be defined. The ompl::base::StateSpace::type_ member must also be set to a new (unique) value when new state types are defined. It will often be necessary to also implement ompl::base::StateSpace::copyState() and perhaps ompl::base::StateSpace::equalStates(). Optionally, if the state type includes real values, an implementation of ompl::base::StateSpace::getValueAddressAtIndex() can be provided to access those values separately. If (de)serialization of states is desired, the ompl::base::StateSpace::serialize() and ompl::base::StateSpace::deserialize() functions must also be implemented.

## Inheriting from ompl::base::CompoundStateSpace

Another option is to inherit from a ompl::base::CompoundStateSpace and call ompl::base::CompoundStateSpace::addSubspace() in the constructor of the new class for other existing state spaces. This is the easiest way to create new state spaces -- only the constructor needs to be provided. For example, see ompl::base::SE2StateSpace. Optionally, the ompl::base::CompoundStateSpace::lock() function can be called after the components have been set in order to prevent the user of the state space from adding further components.

Optionally, if there exist projections to Euclidean spaces (ompl::base::ProjectionEvaluator) for the defined state space, these can be registered by the ompl::base::StateSpace::registerProjections() function (by calling ompl::base::StateSpace::registerProjection() or ompl::base::StateSpace::registerDefaultProjection()). Planners that need a projection but do not have one defined will attempt using this default projection during planning.

\note It is always a good idea to call ompl::base::StateSpace::sanityChecks() to see if a new state space satisfies expected functionality.
