# Working with States and State Spaces {#workingWithStates}

## Allocating memory for states {#stateAlloc}

### The simple version

~~~{.cpp}
ompl::base::StateSpacePtr space(new T());
ompl::base::ScopedState<> state(space);
~~~

or

~~~{.cpp}
ompl::base::SpaceInformationPtr si(space);
ompl::base::ScopedState<T> state(si);
~~~

The ompl::base::ScopedState class will do the necessary memory operations to allocate a state from the correct state space. This is the recommended way of allocating states for code other than ompl internals. Convenience operators such as = and == are provided. If a type T is provided, where T is a state space type, the maintained state is cast as T::StateType. operator= will use ompl::base::StateSpace::copyState() and operator== will use ompl::base::StateSpace::equalStates().

### The expert version

~~~{.cpp}
ompl::base::SpaceInformationPtr si(space);
ompl::base::State* state = si->allocState();
...
si->freeState(state);
~~~

The structure of a state depends on the state space specification. The ompl::base::State type is just an abstract base for the states of other state spaces. For this reason, states cannot be allocated directly, but through the allocation mechanism of the state space: ompl::base::StateSpace::allocState(). States are to be freed using ompl::base::StateSpace::freeState(). For convenience, ompl::base::SpaceInformation::allocState() and ompl::base::SpaceInformation::freeState() are defined as well. Using the calls from the ompl::base::SpaceInformation class is better since they certainly use the same state space as the one used for planning. This is the lowest level of operating on states and only recommended for expert users.

See [Working with states](#stateOps) for how to fill the contents of the allocated states.

## Working with states {#stateOps}

In order for states to be useful in setting start (or goal) positions, accessing their content is needed. It is assumed the reader is familiar with [Allocating memory for states](#stateAlloc). Furthermore, [operators on states and state spaces](#stateAndSpaceOperatorsCopy) are also used.

### Simple version

The recommended use of states is with ompl::base::ScopedState. Given the instance of a state space, this class will allocate a state from that space. The internally maintained state is freed when the instance of ompl::base::ScopedState goes out of scope. ompl::base::ScopedState is a templated class and it inherits from T::StateType, where T is a state space type. This allows it to cast the state it maintains to the desired type and thus exhibit the functionality (the same members) as T::StateType. If no template argument is specified, the internal state is kept as ompl::base::State*.

~~~{.cpp}
ompl::base::StateSpacePtr space(new ompl::base::SE2StateSpace());
ompl::base::ScopedState<ompl::base::SE2StateSpace> state(space);
state->setX(0.1);
state->setY(0.2);
state->setYaw(0.0);

ompl::base::ScopedState<> backup = state;
// backup maintains its internal state as State*, so setX() is not available.
// the content of backup is copied from state

ompl::base::State *abstractState = space->allocState();


// this will copy the content of abstractState to state and
// cast it internally as ompl::base::SE2StateSpace::StateType
state = abstractState;

// restore state to it's original value
state = backup;

if (state != backup)
   throw ompl::Exception("This should never happen");
~~~

Combining ompl::base::ScopedState with ompl::base::CompoundStateSpace:

~~~{.cpp}
ompl::base::CompoundStateSpace *cs = new ompl::base::CompoundStateSpace();
cs->addSubspace(ompl::base::StateSpacePtr(new ompl::base::SO2StateSpace()), 1.0);
cs->addSubspace(ompl::base::StateSpacePtr(new ompl::base::SO3StateSpace()), 1.0);

// put the pointer to the state space in a shared pointer
ompl::base::StateSpacePtr space(cs);

// the ompl::base::ScopedState helps only with one cast here, since we still need to
// manually cast the components of the state to what we want them to be.
ompl::base::ScopedState<ompl::base::CompoundStateSpace> state(space);
state->as<ompl::base::SO2StateSpace::StateType>(0)->setIdentity();
~~~

The code above can be equivalently written as:

~~~{.cpp}
// define the individual state spaces
ompl::base::StateSpacePtr so2(new ompl::base::SO2StateSpace());
ompl::base::StateSpacePtr so3(new ompl::base::SO3StateSpace());

// construct a compound state space using the overloaded operator+
ompl::base::StateSpacePtr space = so2 + so3;

// the ompl::base::ScopedState helps only with one cast here, since we still need to
// manually cast the components of the state to what we want them to be.
ompl::base::ScopedState<ompl::base::CompoundStateSpace> state(space);
state->as<ompl::base::SO2StateSpace::StateType>(0)->setIdentity();
~~~

States can also be printed to streams:

~~~{.cpp}
ompl::base::ScopedState<> state(space);
std::cout << state;
~~~

Sometimes it may be useful to extract parts of a state, or assign only
to some parts of a state, especially when using compound state spaces:

~~~{.cpp}
// an SE2 state space is in fact a compound state space consisting of R^2 and SO2
ompl::base::StateSpacePtr space(new ompl::base::SE2StateSpace());
// define a full state for this state space
ompl::base::ScopedState<ompl::base::SE2StateSpace> fullState(space);
// set the state to a random value
fullState.random();

// construct a state that corresponds to the position component of SE2
ompl::base::ScopedState<> pos(space->as<ompl::base::SE2StateSpace>()->getSubspace(0));

// copy the position
pos << fullState;

// equivalently, this can be done too:
fullState >> pos;

// if we now modify pos somehow, we can set it back in the full state:
pos >> fullState;
~~~

## Expert version

For a state space type of type T, the result of ompl::base::StateSpace::allocState() can be casted to T::StateType* to gain access to the state's members. To ease this functionality, the ompl::base::State::as() functions have been defined.

~~~{.cpp}
ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(1));
ompl::base::State *state = space->allocState();
state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = 0.1;
ompl::base::State *copy = space->allocState();
space->copyState(copy, state);
if (!space->equalStates(copy, state))
   throw ompl::Exception("This should not happen");
space->freeState(state);
space->freeState(copy);
~~~

See [Advanced methods for copying states](group__advancedStateCopy.html) for more information on copying states.

## Operators for States and State Spaces {#stateAndSpaceOperatorsCopy}

\copydoc stateAndSpaceOperators
