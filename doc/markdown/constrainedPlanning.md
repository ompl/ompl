# Constrained Planning {#constrainedPlanning}

Sometimes, the motion of a robot is _constrained_ by its task or inherent kinematic properties. For example, a robot might want to keep a cup of water level, remain in contact with a surface to write on it, or follow some curve in space that corresponds to a task. In each of these cases, the constraint on the robot's motion is defined by some _function_, \f$f(q) : \mathcal{Q} \rightarrow \mathbb{R}^n\f$, which maps the robot's state space \f$\mathcal{Q}\f$ onto a real vector value \f$\mathbb{R}^n\f$. The constraint is considered satisfied whenever \f$f(q) = \mathbf{0}\f$. For example, to keep the cup level, the angular distance of the cup's axis to the upright z-axis could be the constraint function. In addition, a constraint satisfying motion satisfies \f$f(q) = 0\f$ at every point along the path. Constrained motion planning addresses the question of how to find a constraint satisfying motion, while still avoiding obstacles or achieving other objectives.

In general, motion planners are not aware of constraints, and will generate paths that do not satisfy the constraint. This is because the _submanifold_ of constraint satisfying configurations \f$X = \{ q \in \mathcal{Q} \mid f(q) = \mathbf{0} \}\f$ is lower-dimensional compared to the state space of the robot, and thus near impossible to sample from. To plan constraint satisfying motion, OMPL provides a means to augment the state space of a robot with knowledge of the constraint, representing $X$ as a state space. Any motion plan generated using this augmented state space will satisfy the constraint, as the primitive operations used by motions planners (e.g., `ompl::StateSpace::interpolate`) automatically generate constraint satisfying states. The constrained planning framework enables any sampling-based planner (included asymptotically optimal planners) to plan while respecting a constraint function.

You can represent a constraint function using `ompl::base::Constraint`, where you must implement the function, and possibly the analytic Jacobian of the constraint function. By default, if no analytic Jacobian is provided, a numerical finite central difference routine is used to approximate the Jacobian. However, this is very computationally intensive, and providing an analytic derivative is preferred. We provide a simple script `ConstraintGeneration.py` that uses the [SymPy](http://www.sympy.org/en/index.html) Python library for symbolic differentiation of constraints, and can automatically generate constraint code that can be used in your programs. There is also `ompl::base::ConstraintIntersection`, which allows for composition of multiple constraints together that must all be satisfied.

There are currently three provided representations of constrained state spaces, each of which inherits from `ompl::base::ConstrainedStateSpace`. Each of these methods implements a different way to sample and traverse the underlying submanifold. The three augmented state spaces are:

 - `ompl::base::ProjectedStateSpace`, a constrained state space that uses a projection operator to find constraint satisfying motion.
 - `ompl::base::AtlasStateSpace`, a constrained state space that, while planning, builds a piecewise linear approximation of $X$ using tangent spaces. This approximation is called an _atlas_, and is used to guide planning.
 - `ompl::base::TangentBundleStateSpace`, a constrained state space similar to `ompl::base::AtlasStateSpace`, but lazily evaluates the atlas.

Each of these state spaces has their own documentation that can be viewed on their page. The augmented state space approach taken by OMPL was presented in [this paper](http://kavrakilab.org/publications/kingston2017decoupling-constraints.pdf). More information about constrained motion planning is presented in [this review paper](http://kavrakilab.org/publications/kingston2018sampling-based-methods-for-motion-planning.pdf).

You can view an example of how to use the constrained planning framework in [this tutorial](constrainedPlanningTutorial.html)!

## Limitations

In order for the constrained planning framework to work, your underlying state space and constraint function must satisfy a few assumptions.

#### Contiguous Memory

As an implementation detail, the memory allocated in a state by the underlying state space must be a contiguous array of `double` values. For example, `ompl::base::RealVectorStateSpace`, or the state space implemented in the kinematic chain benchmark both allocate a contiguous array of `double` values. This detail is required by the various constrained state spaces and constraint function in order to view them as `Eigen::VectorXd`s, using `Eigen::Map<Eigen::VectorXd>`. Note that `ompl::base::ConstrainedStateSpace::StateType` derives from this class, for convience.

However, this assumption prevents the `ompl::base::CompoundStateSpace` from being used by the constrained planning framework, as it does not guarantee contiguity.

#### Constraint Differentiability

In general, your constraint function should be a _continuous_ and _differentiable_ function of the robot's state. Singularities in the constraint function can cause bad behavior by the underlying constraint satisfaction methods.

#### Hyperparameter Sensitivity

The constrained state spaces, in general, are sensitive to the tuning of their various hyperparameters. Some reasonable defaults are set at start, but many constrained planning problems will have different characteristics of the underlying submanifold, and as such different parameters may be needed. Some basic rules-of-thumb are provided below:

 - For high-dimensional ambient state spaces, many parameters can be increased in magnitude. A brief non-exhaustive list follows.
  - Constraint solving tolerance can be increased with `ompl::base::Constraint::setTolerance()`
  - Valid step size for manifold traversal with `ompl::base::ConstrainedStateSpace::setDelta()`
  - Many atlas related parameters for `ompl::base::AtlasStateSpace`, such as \f$\epsilon, \rho\f$, and exploration.
 - Generally, the step size for manifold traversal is related to the _curvature_ of the underlying constraint submanifold. Less curvy submanifolds can permit larger step sizes. Play around with this value if speed is a concern, as the larger the step size is, the less time is spent traversing the manifold.
 - For the atlas- and tangent bundle-based spaces, planners that rely on uniform sampling of the space may require a high exploration parameter so the constraint submanifold becomes covered quicker (e.g., BIT*). Additionally, planners that rely on `ompl::base::StateSampler::samplerNear()` may also require a high exploration or $\rho$ parameter in order to allow for faster search.
 - And many others! The projection-based space is less sensitive to poor parameter tunings than the atlas- or tangent bundle-based space in general, and as such is a good starting point to validate whether a constrained planning problem is feasible.

## Want to learn more?

Check out the [tutorial](constrainedPlanningTutorial.html), and the following demos:
 - [ConstrainedPlanningSphere](ConstrainedPlanningSphere_8cpp_source.html). Plan for a point in R<sup>3</sup> constrained to the surface of a sphere, with narrow obstacles it must traverse.
 - [ConstrainedPlanningImplicitChain](ConstrainedPlanningImplicitChain_8cpp_source.html). Plan for a set of balls, each in R<sup>3</sup>, constrained to be a unit distance apart. This imposes spherical kinematics on the balls, implicitly defining a kinematic chain.
 - [ConstrainedPlanningKinematicChain](ConstrainedPlanningKinematicChain_8cpp_source.html). Similar to the kinematic chain benchmark demo, but with a constraint that only allows the tip of the manipulator to move along a line.