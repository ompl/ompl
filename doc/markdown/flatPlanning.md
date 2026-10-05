# Planning for Differentially Flat Systems {#flatPlanning}

[TOC]

Planning with kinodynamic constraints is often much slower than geometric planning.
The high dimensionality of the space, combined with a common inability to directly connect two robot states with an edge, means that kinodynamic planners struggle to perform well on hard problems, especially for manipulators.
This tutorial gives a walkthrough of planning for kinodynamic systems via _differential flatness_, a concept that allows geometric motion planners to solve kinodynamic planning problems.

We say a system is _differentially flat_ if we can describe a system's state through its _flat output_: a set of independent variables that, together with their derivatives, determines exactly the system's state and its applied control inputs.
We call the combination of a flat output and its derivatives a _flat state_.
For a robot arm driven by joint torques, the flat output is its joint angles.
For a quadrotor, the flat output is the drone's Cartesian position and its yaw angle, since we can compute the drone's attitude and thrust from just these values.

Using `ompl::base::FlatStateSpace`, users can use geometric planners to find kinodynamically valid plans for differentially flat systems.
Each point in a `FlatStateSpace` is a single flat state.
For any flat system, we can interpolate between two flat states with a polynomial, so `FlatStateSpace` looks like a geometric state space to a planner, even though it respects kinodynamic constraints.

Once a planner has solved a problem, you can build a `ompl::base::FlatTrajectory` from its solution path and use it to generate controller inputs for the system.

The approach to flat planning described here follows FLASK, described in T. Duong, C. W. Ramsey, Z. Kingston, W. Thomason, and L. E. Kavraki, [Ultrafast sampling-based kinodynamic planning via differential flatness](https://arxiv.org/abs/2603.16059), <em>IEEE Transactions on Robotics</em>, 2026.

## From dynamics to a planner

Take a system \f$\dot{x} = f(x, u)\f$ with state \f$x\f$ and input \f$u\f$.
It's flat when some output \f$y\f$, with as many components as \f$u\f$, determines both through its first \f$k\f$ derivatives.
In other words, if we know \f$y\f$, we may calculate the state \f$x\f$ through some function \f$\phi\left(y, \dot{y}, \ldots, y^{(k-1)}\right)\f$, and likewise the input \f$u\f$ through \f$\psi\left(y, \dot{y}, \ldots, y^{(k)}\right)\f$.

\f[
x = \phi\left(y, \dot{y}, \ldots, y^{(k-1)}\right), \qquad u = \psi\left(y, \dot{y}, \ldots, y^{(k)}\right)
\f]

Once you have an arbitrary flat system, you can plan with it in OMPL as follows:

1. Find \f$y\f$, \f$\phi\f$, and \f$\psi\f$.
   OMPL never sees \f$\phi\f$ or \f$\psi\f$ directly.
   You apply them yourself, in the state validity checker and when reading inputs off the solution.
2. Pick the OMPL state space that contains the flat output \f$y\f$.
   Real vectors, SO(2), and compounds of them work out of the box.
3. Build a `FlatStateSpace` over the flat output space with order \f$k\f$.
4. Write the constraints on \f$x\f$ in terms of the flat state.
   Independent limits on each component of a derivative, such as a velocity cap per joint, become derivative bounds.
   Everything else, such as collisions or a cap on total speed, goes in the state validity checker, which receives the flat state and can apply \f$\phi\f$ itself.

Every motion connecting two flat states is a smooth polynomial in \f$y\f$, and \f$\phi\f$ and \f$\psi\f$ turn any smooth \f$y(t)\f$ into a state and input that satisfy the dynamics, so every edge is dynamically feasible.
The input depends on \f$y^{(k)}\f$, which isn't stored in flat states but can be read off the motion between them.
To find the input at time \f$t\f$, call `ompl::base::FlatTrajectory::evaluate` at \f$t\f$ once for each derivative level from 0 through \f$k\f$, then pass the results to \f$\psi\f$.
Input limits depend on \f$y^{(k)}\f$ as well, so check them against the trajectory after solving.
The quadrotor planning demo ([FlatQuadrotorPlanning.cpp](FlatQuadrotorPlanning_8cpp_source.html)) does this, reading the acceleration off its trajectory and turning it into thrust and tilt.

### Picking the order

The _order_ of a flat state space is \f$k\f$, the number of derivative levels each flat state holds, counting the flat output itself as level 0.
An order 2 space stores positions and velocities, and an order 3 space adds accelerations.
If you command accelerations, choose order 2, and if you command jerk, choose order 3.
Some examples:

- An arm driven by joint torques or a mobile base taking acceleration commands has order 2.
- A quadrotor commanded by collective thrust and attitude sets its acceleration directly, so order 2 works there too.
- A quadrotor commanded by thrust and body rates sets its jerk, which has order 3.
- A quadrotor commanded by rotor torques sets its snap, which has order 4.

Derivative levels below the order are continuous along a solved trajectory.
Levels at or above it jump where one edge hands over to the next.

Higher orders cost more.
For a system of order \f$k\f$, interpolating between two states requires solving a polynomial system of order \f$2k\f$.
For \f$k=2\f$, this is cheap, since quartics have an analytic solution, but higher-order systems are orders of magnitude less efficient.

### Robot arms

A fully actuated arm with joint angles \f$\theta\f$ and joint torques \f$\tau\f$ follows the manipulator equation.

\f[
M(\theta)\ddot{\theta} + C(\theta, \dot{\theta})\dot{\theta} + g(\theta) = \tau
\f]

Every joint has its own motor, so \f$\theta\f$ is a flat output, and the torque sets \f$\ddot{\theta}\f$, so the order is 2.
The flat state \f$(\theta, \dot{\theta})\f$ is the system state, so \f$\phi\f$ is the identity, and the left-hand side above is \f$\psi\f$.

[FlatManipulatorPlanning.cpp](FlatManipulatorPlanning_8cpp_source.html) plans a Panda in this space.
The solution comes back as \f$\theta(t)\f$ with continuous velocity and piecewise polynomial acceleration.
Finally, the demo evaluates the joint velocities at 20,001 evenly spaced times along the trajectory and reports the fastest speed of each joint against its velocity cap.

## Setting up a flat state space

This walkthrough follows a simplified version of [FlatManipulatorPlanning.cpp](FlatManipulatorPlanning_8cpp_source.html), which plans a Panda arm through a cage of spheres with [VAMP](https://github.com/KavrakiLab/vamp) checking collisions.
The flat output of the arm is its joint configuration, so the flat output space is the Panda's joint space.

```{.cpp}
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace ov = ompl::vamp;
using Panda = vamp::robots::Panda;

// Order 2 holds the joint angles and the joint velocities.
auto space = std::make_shared<ob::FlatStateSpace>(std::make_shared<ov::VampStateSpace<Panda>>(), 2);

// One sphere for the arm to avoid.
vamp::collision::Environment<float> spheres;
spheres.spheres.push_back(vamp::collision::factory::sphere::array({0.55f, 0.f, 0.25f}, 0.2f));
spheres.sort();
const ov::VampStateValidityChecker<Panda>::Environment environment(spheres);

auto outputInfo = std::make_shared<ob::SpaceInformation>(space->getOutputSpace());
auto collision = std::make_shared<ov::VampStateValidityChecker<Panda>>(outputInfo, environment);

og::SimpleSetup setup(space);
auto si = setup.getSpaceInformation();
setup.setStateValidityChecker(
    [space, collision](const ob::State *state)
    {
        return space->satisfiesBounds(state) &&
               collision->isValid(state->as<ob::FlatStateSpace::StateType>()->output());
    });
setup.setOptimizationObjective(std::make_shared<ob::FlatEffortObjective>(si));
```

Above is a simplified version of the setup for planning in a flat system.
We construct a flat state space of order 2, meaning that flat states in this problem contain only positions and velocities.
Next, we construct a collision-checking environment and a VAMP validity checker that tests the robot's joint angles against it.
Any validity checker over the flat output space can stand in for `collision`.

Finally, we create a `SimpleSetup` structure that contains the validity checker, space, and optimization objective (for reporting path costs).
A polynomial between two flat states can overshoot, so it can leave the bounds of the space even when both of its endpoints are in bounds.
The `satisfiesBounds` call rejects every sampled state along an edge that leaves the bounds.

Next, we apply velocity limits:

```{.cpp}
// The Panda's velocity limit at each joint, in radians per second.
ob::RealVectorBounds speeds(Panda::dimension);
speeds.high = {2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61};
for (unsigned int i = 0; i < Panda::dimension; ++i)
    speeds.low[i] = -speeds.high[i];
space->getDerivativeSpace(1)->setBounds(speeds);
```

Any limit that isn't a per-component box can go in the state validity checker, which rejects states that break it.
To also change how states get sampled and clamped, pass your own derivative spaces to the second `FlatStateSpace` constructor.

Flat states are stored as `ompl::base::FlatStateSpace::StateType`.
`fromFlatState` and `toFlatState` convert between a flat state and a matrix with one row per derivative level and one column per flat output variable.

```{.cpp}
// The arm starts and finishes at rest.
Eigen::MatrixXd flat = Eigen::MatrixXd::Zero(2, Panda::dimension);
ob::ScopedState<> start(space), goal(space);
flat.row(0) << 0., -0.785, 0., -2.356, 0., 1.571, 0.785;
space->fromFlatState(flat, start.get());
flat.row(0) << 2.35, 1., 0., -0.8, 0., 2.5, 0.785;
space->fromFlatState(flat, goal.get());
setup.setStartAndGoalStates(start, goal);
```

You can also read the parts of a flat state through the `output()` and `derivative(level)` methods of `StateType`.

## Edge costs and distance

The process of finding a valid interpolation between two flat states is called _steering_.
For two given endpoints, steering finds the polynomial between them with the lowest cost.
The cost of an edge lasting \f$T\f$ is its control effort, \f$\int_0^T \lVert y^{(k)}(t) \rVert^2 \, dt\f$, plus a time penalty of \f$\rho T\f$.
Raising \f$\rho\f$ buys faster edges that spend more effort.
You can change \f$\rho\f$ via `ompl::base::FlatStateSpace::setRho`.

`ompl::base::FlatEffortObjective` charges exactly that cost per edge.
The space warns when an optimizing planner runs without it, because the default objective sums distances between flat states, which says little about the cost of traversing a path through a flat system.
The objective's cost-to-go estimate never overestimates, so planners that use cost heuristics can rely on it.

Since computing the effort objective takes a steering solve, we use a simpler distance function for nearest-neighbor queries.
By default, the nearest-neighbor search distance function is the distance in the flat output space plus a weighted Euclidean distance at each derivative level, with each component weighted by the reciprocal of its extent.
That distance is a metric and needs no steering, so planners keep their fast nearest-neighbor structures.
Calling `ompl::base::FlatStateSpace::setDistanceType` with `TRAJECTORY_COST` makes distance the edge cost instead, which runs one way and costs a steering solve per query, so nearest-neighbor searches have to check every state in the tree.
Set the distance type before the planner sets up.

## Checking edges

`ompl::base::SpaceInformation` installs `ompl::base::FlatMotionValidator` for a flat state space automatically.
It solves each edge once and checks samples along the polynomial, spaced so the flat output moves at most a fixed fraction of the extent of the flat output space between samples.
To adjust the spacing of samples, you can use `ompl::base::StateSpace::setLongestValidSegmentFraction` on the flat state space, or use `ompl::base::SpaceInformation::setStateValidityCheckingResolution`.
The fraction also appears in the parameter set of the space as `longest_valid_segment_fraction`, so a benchmark can sweep it.

For robots supported by VAMP, you can check edges with `ompl::vamp::FlatVampMotionValidator`, which evaluates a whole batch of samples in one SIMD collision check.

```{.cpp}
si->setMotionValidator(std::make_shared<ov::FlatVampMotionValidator<Panda>>(si, environment));
```

This motion validator rejects any sample where the robot collides or any derivative level leaves its bounds.

## Flat outputs with topology

In order to construct the interpolating polynomial between flat states, we need to construct a local coordinate space.
Given a start state for interpolation, `ompl::base::FlatChart` builds a local coordinate system centered on that state, and then generates the steering polynomial in that frame.
`ompl::base::allocFlatChart` builds a chart for anything deriving from `ompl::base::RealVectorStateSpace` or `ompl::base::SO2StateSpace` and for compounds of them, which covers SE(2), tori, and the quadrotor's position and yaw.
For custom flat output spaces, you have to manually create a chart structure.

For example, for a quadrotor system, we can describe its flat output from its position, a three-dimensional real vector; and its yaw, a rotation in \f$SO(2)\f$.
Constructing a flat state space from this compound state space "just works":

```{.cpp}
// The quadrotor's flat output is its position in a 10 meter room and its yaw.
auto position = std::make_shared<ob::RealVectorStateSpace>(3);
position->setBounds(-5., 5.);
auto pose = std::make_shared<ob::CompoundStateSpace>();
pose->addSubspace(position, 1.);
pose->addSubspace(std::make_shared<ob::SO2StateSpace>(), 0.5);
auto quadrotor = std::make_shared<ob::FlatStateSpace>(pose, 2);
```

A chart of your own derives from `ompl::base::FlatChart` and implements `difference`, which writes the coordinates of one state in the chart centered on another, and `advance`, which maps coordinates back to a state.
Setting an entry of `periods_` marks that coordinate as wrapping with that period.
Steering treats the chart as exact, so a chart over a curved space such as SO(3) makes each edge an approximation of the motion that it stands for.

## Choosing a planner

Given two states, \f$a\f$ and \f$b\f$, the edge running from \f$a\f$ to \f$b\f$ is distinct from the edge running from \f$b\f$ to \f$a\f$.
This means that flat state spaces are asymmetric, and a planner that traverses an edge backward returns a path that fails `ompl::geometric::PathGeometric::check()`.
For instance, PRM treats its roadmap edges as bidirectional, so it can't be used with flat state spaces.

Use a planner whose `ompl::base::PlannerSpecs::directed` is set, which means it checks every edge in the direction its path runs.
`RRT`, `RRTConnect`, `BiTRRT`, `EST`, `KPIECE1`, `RRTstar`, and `FMT` all qualify.

## Following the solution

Once you have solved a problem, a `ompl::base::FlatTrajectory` gives you the flat output and its derivatives at any time.

```{.cpp}
setup.setPlanner(std::make_shared<og::RRTConnect>(si));
setup.solve(1.);

const ob::FlatTrajectory trajectory(setup.getSolutionPath());
Eigen::VectorXd velocity(Panda::dimension);
trajectory.evaluate(0.5 * trajectory.duration(), 1, velocity);
```

`evaluate` reads any derivative level at any time, including levels above the system's order.
Evaluating trajectories accumulates across wrapping, so an angle keeps turning past a half turn instead of jumping back, and a controller tracking it sees a smooth signal.
`ompl::base::FlatTrajectory::toState` writes a flat state at any time, wrapped back into the space.

Before constructing a trajectory, you can simplify a solution path with `ompl::geometric::PathSimplifier::reduceVertices`, which re-steers between intermediate states in the solution.

## Python

The Python bindings cover the flat state space, its charts, the effort objective, and trajectories.
[FlatQuadrotorPlanning.py](FlatQuadrotorPlanning_8py_source.html) and [FlatManipulatorPlanning.py](FlatManipulatorPlanning_8py_source.html) port the two demos.

```{.py}
import numpy as np
from ompl import base as ob
from ompl import geometric as og

output = ob.RealVectorStateSpace(2)
bounds = ob.RealVectorBounds(2)
bounds.setLow(-1.0)
bounds.setHigh(1.0)
output.setBounds(bounds)

space = ob.FlatStateSpace(output, 2)
space.setDerivativeBound(1, 2.0)

setup = og.SimpleSetup(space)
setup.setStateValidityChecker(lambda state: space.satisfiesBounds(state))
setup.setOptimizationObjective(ob.FlatEffortObjective(setup.getSpaceInformation()))

start, goal = space.allocState(), space.allocState()
space.fromFlatState(np.array([[-0.8, -0.8], [0.0, 0.0]]), start)
space.fromFlatState(np.array([[0.8, 0.8], [0.0, 0.0]]), goal)
setup.setStartAndGoalStates(start, goal)
setup.setPlanner(og.RRTConnect(setup.getSpaceInformation()))

if setup.solve(1.0):
    trajectory = ob.FlatTrajectory(setup.getSolutionPath())
    print(trajectory.duration(), trajectory.evaluate(0.5 * trajectory.duration(), 1))
```

`toFlatState` returns a NumPy array, and `fromFlatState` takes one.
A Python class can subclass `ompl::base::FlatChart` and pass itself to the constructor, and it calls `setPeriods` to mark wrapping coordinates.
