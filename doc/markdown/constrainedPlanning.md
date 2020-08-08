# Constrained Planning {#constrainedPlanning}

[TOC]

Sometimes, the motion of a robot is _constrained_ by its task or inherent kinematic properties. For example, a robot might want to keep a cup of water level, remain in contact with a surface to write on it, or follow some curve in space that corresponds to a task. In each of these cases, the constraint on the robot's motion is defined by some _function_, \f$f(q) : \mathcal{Q} \rightarrow \mathbb{R}^n\f$, which maps the robot's state space \f$\mathcal{Q}\f$ onto a real vector value \f$\mathbb{R}^n\f$. The constraint is considered satisfied whenever \f$f(q) = \mathbf{0}\f$. For example, to keep the cup level, the angular distance of the cup's axis to the upright z-axis could be the constraint function. In addition, a constraint satisfying motion satisfies \f$f(q) = \mathbf{0}\f$ at every point along the path. Constrained motion planning addresses the question of how to find a constraint satisfying motion, while still avoiding obstacles or achieving other objectives.

In general, motion planners are not aware of constraints, and will generate paths that do not satisfy the constraint. This is because the _submanifold_ of constraint satisfying configurations \f$X = \{ q \in \mathcal{Q} \mid f(q) = \mathbf{0} \}\f$ is lower-dimensional compared to the state space of the robot, and thus near impossible to sample from. To plan constraint satisfying motion, OMPL provides a means to augment the state space of a robot with knowledge of the constraint, representing \f$X\f$ as a state space. Any motion plan generated using this augmented state space will satisfy the constraint, as the primitive operations used by motions planners (e.g., `ompl::base::StateSpace::interpolate`) automatically generate constraint satisfying states. The constrained planning framework enables any sampling-based planner (included asymptotically optimal planners) to plan while respecting a constraint function.

You can represent a constraint function using `ompl::base::Constraint`, where you must implement the function \f$f\f$, and optionally the analytic Jacobian of the constraint function. If no analytic Jacobian is provided, a numerical finite central difference routine is used to approximate the Jacobian. However, this is very computationally intensive, and providing an analytic derivative is preferred. We provide a simple script `ConstraintGeneration.py` that uses the [SymPy](https://www.sympy.org/en/index.html) Python library for symbolic differentiation of constraints, and can automatically generate constraint code that can be used in your programs. There is also `ompl::base::ConstraintIntersection`, which allows for composition of multiple constraints together that must all be satisfied.

There are currently three provided representations of constrained state spaces, each of which inherits from `ompl::base::ConstrainedStateSpace`. Each of these methods implements a different way to sample and traverse the underlying submanifold. The three augmented state spaces are:

- `ompl::base::ProjectedStateSpace`, a constrained state space that uses a projection operator to find constraint satisfying motion.
- `ompl::base::AtlasStateSpace`, a constrained state space that, while planning, builds a piecewise linear approximation of \f$X\f$ using tangent spaces. This approximation is called an _atlas_, and is used to guide planning.
- `ompl::base::TangentBundleStateSpace`, a constrained state space similar to `ompl::base::AtlasStateSpace`, but lazily evaluates the atlas.

Each of these state spaces has their own documentation that can be viewed on their page. The augmented state space approach taken by OMPL was presented in [a 2019 paper in the International Journal of Robotics Research](https://www.kavrakilab.org/publications/kingston2019exploring-implicit-spaces-for-constrained.pdf). More information about constrained motion planning is presented in [this review paper](http://kavrakilab.org/publications/kingston2018sampling-based-methods-for-motion-planning.pdf).

You can view an example of how to use the constrained planning framework in [this tutorial](constrainedPlanningTutorial.html)!

## Limitations

In order for the constrained planning framework to work, your underlying state space and constraint function must satisfy a few assumptions.

### Contiguous Memory

As an implementation detail, the memory allocated in a state by the underlying state space must be a contiguous array of `double` values. For example, `ompl::base::RealVectorStateSpace`, or the state space implemented in the kinematic chain benchmark both allocate a contiguous array of `double` values. This detail is required by the various constrained state spaces and constraint function in order to view them as `Eigen::VectorXd`s, using `Eigen::Map<Eigen::VectorXd>`. Note that `ompl::base::ConstrainedStateSpace::StateType` derives from this class, for convenience.

However, this assumption prevents the `ompl::base::CompoundStateSpace` from being used by the constrained planning framework, as state allocation does not guarantee contiguity.

### Constraint Differentiability

In general, your constraint function should be a _continuous_ and _differentiable_ function of the robot's state. Singularities in the constraint function can cause bad behavior by the underlying constraint satisfaction methods. `ompl::base::AtlasStateSpace` and `ompl::base::TangentBundleStateSpace` both will treat singularities as obstacles in the planning process.

### Required Interpolation

If you want a path that satisfies constraints _and_ is potentially executable by a real system, you will want to interpolate whatever path (simplified or un-simplified) you find.
This can be done by simply calling `ompl::geometric::PathGeometric::interpolate()`.
The interpolated path will be constraint satisfying, as the interpolate routine uses the primitives afforded by the constrained state space.
However, there can potentially be issues at this step, as interpolation can fail.

### Interpolation Failures

Currently, each of the constrained state spaces implements interpolation on the constraint submanifold via computing a _discrete geodesic_ between the two query points. The discrete geodesic is a sequence of close (to approximate continuity), constraint satisfying states between two query points. The distance between each point in the discrete geodesic is tuned by the "delta" parameter in `ompl::base::ConstrainedStateSpace::setDelta()`. How this discrete geodesic is computed is key to how constrained state space operates, as it is used ubiquitously throughout the code (e.g., interpolation, collision checking, motion validation, and others).

Due to the nature of how these routines are implemented, it is possible for computation of the discrete geodesic to _fail_, thus causing potentially unexpected results from whatever overlying routine requested a discrete geodesic. These failures can be the result of singularities in the constraint, high curvature of the submanifold, and various other issues. However, interpolation in "regular" state spaces does not generally fail as they are analytic, such as linear interpolation in `ompl::base::RealVectorStateSpace`; hence, `ompl::base::StateStace::interpolate()` is assumed to always be successful. As a result, some unexpected behavior can be seen if interpolation fails during planning with a constrained state space. Increasing or decreasing the "delta" parameter in `ompl::base::ConstrainedStateSpace`, increasing or decreasing the constraint satisfaction tolerance, and other hyperparameter tuning can fix these problems.

### Hyperparameter Sensitivity

The constrained state spaces, in general, are sensitive to the tuning of their various hyperparameters. Some reasonable defaults are set at start, but many constrained planning problems will have different characteristics of the underlying submanifold, and as such different parameters may be needed. Some basic rules-of-thumb are provided below:

- For high-dimensional ambient state spaces, many parameters can be increased in magnitude. A brief non-exhaustive list follows.
  - Constraint solving tolerance can be increased with `ompl::base::Constraint::setTolerance()`
  - Valid step size for manifold traversal with `ompl::base::ConstrainedStateSpace::setDelta()`
  - Many atlas related parameters for `ompl::base::AtlasStateSpace`, such as \f$\epsilon, \rho\f$, and exploration.
- Generally, the step size for manifold traversal is related to the relative _curvature_ of the underlying constraint submanifold. Less curvy submanifolds can permit larger step sizes (i.e., if the constraint defines a hyperplane, you can use a large step size). Play around with this value if speed is a concern, as the larger the step size is, the less time is spent traversing the manifold.
- For the atlas- and tangent bundle-based spaces, planners that rely on uniform sampling of the space may require a high exploration parameter so the constraint submanifold becomes covered quicker (e.g., BIT*). Additionally, planners that rely on `ompl::base::StateSampler::samplerNear()` may also require a high exploration or \f$\rho\f$ parameter in order to expand effectively.
- And many others! In general, the projection-based space is less sensitive to poor parameter tuning than atlas- or tangent bundle-based space, and as such is a good starting point to validate whether a constrained planning problem is feasible.

## Additional Notes

### Constraint Projection versus Projection Evaluator

Within `ompl::base::Constraint`, there is a projection function `ompl::base::Constraint::project()` which maps a potentially constraint unsatisfying configuration to the constraint manifold.
By default, `ompl::base::Constraint::project()` implements a Newton's method which performs well in most circumstances.
Note that it is possible to override this method with your own projection routine, e.g., inverse kinematics.

You might notice that there also exists `ompl::base::ProjectionEvaluator::project()`
This method is used by a few planners to estimate the coverage of the free space (see [this tutorial for more information](projections.html)), as it is a "projection" into a lower-dimensional space.
Although similar sounding, this concept is orthogonal to the concept of projection as used in `ompl::base::Constraint`, which projects states into the lower-dimensional constraint manifold.
In fact, one can use planners that use a `ompl::base::ProjectionEvaluator` (e.g., `ompl::geometric::KPIECE1`, `ompl::geometric::ProjEST`) in tandem with the constrained planning framework.

By default, the constrained state spaces will use `ompl::base::WrapperProjectionEvaluator` to access the underlying state space's default projection evaluator.
However, if you know anything about the structure of you problem, you should implement your own projection evaluator for performance.
Each of the constrained planning demos has an example of a projection evaluator that can be used with constrained planning (e.g., `SphereProjection`, in [ConstrainedPlanningSphere](ConstrainedPlanningSphere_8cpp_source.html)).

## Want to learn more?

### Tutorials

Check out the [tutorial](constrainedPlanningTutorial.html), which shows how to set up a constrained planning problem in R<sup>3</sup>.

### Demos

The examples (plus more) from [the paper](http://www.kavrakilab.org/publications/kingston2019exploring-implicit-spaces-for-constrained.pdf) that presented the constrained planning framework are available as demo programs. All of the demos are in the `ompl/demos/constraint` folder. Each of these demos supports planning for an individual planner as well as benchmarking, and complete configurability of the hyperparameters of the constrained space. Each of these demos comes with a way to visualize the results of planning. The sphere, torus, and implicit kinematic chain demos each come with a [Blender](https://www.blender.org/) file (`.blend`) that visualize the results of planning in 3-D. These are all in `ompl/demos/constraint/visualization`.

- [ConstrainedPlanningSphere](ConstrainedPlanningSphere_8cpp_source.html) [[Python version]](ConstrainedPlanningSphere_8py_source.html). Plan for a point in R<sup>3</sup> constrained to the surface of a sphere, with narrow obstacles it must traverse. A similar scenario is explained in the [tutorial](constrainedPlanningTutorial.html). The Blender file `ConstrainedPlanningSphere.blend` contains a script that visualized the motion graph, original and simplified paths, and the atlas generated while planning if `ompl::base::AtlasStateSpace` or `ompl::base::TangentBundleStateSpace` were used. Simply change the path in the script window to the directory where the output of the demo was placed and hit "Run Script". The Python module [NetworkX](https://networkx.github.io/) is required to visualize the graph, make sure your Blender's Python path is correctly set so it can find the module. Note that you must run the demo with `-o` to dump output.
- [ConstrainedPlanningTorus](ConstrainedPlanningTorus_8cpp_source.html) [[Python version]](ConstrainedPlanningTorus_8py_source.html). Plan for a point in R<sup>3</sup> constrained to the surface of a torus. A "maze" image is loaded (some examples are provided in `ompl/demos/constraint/mazes`) to use as the set of obstacles to traverse. The start and goal point are red and green pixels in the images respectively. The Blender file `ConstrainedPlanningTorus.blend` contains a script to visualize the motion graph, simplified path, the atlas generated by planning, and the maze projected upon the torus. Similar to the sphere example, change the path in the script window and "Run Script" to see results.
- [ConstrainedPlanningImplicitChain](ConstrainedPlanningImplicitChain_8cpp_source.html) [[Python version]](ConstrainedPlanningImplicitChain_8py_source.html). Plan for a set of balls, each in R<sup>3</sup>, constrained to be a unit distance apart. This imposes spherical kinematics on the balls, implicitly defining a kinematic chain. The Blender file `ConstrainedPlanningImplicitChain.blend` contains a script to show an animation of the simplified path . Similar to the sphere example, change the path in the script window and "Run Script" to see results.
- [ConstrainedPlanningImplicitParallel](ConstrainedPlanningImplicitChain_8cpp_source.html). Plan for a parallel robot made of many implicit chains. The Blender file `ConstrainedPlanningImplicitParallel.blend` contains a script to show an animation of the simplified path. Similar to the sphere example, change the path in the script window and "Run Script" to see results.
- [ConstrainedPlanningKinematicChain](ConstrainedPlanningKinematicChain_8cpp_source.html). Similar to the kinematic chain benchmark demo, but with a constraint that only allows the tip of the manipulator to move along a line.
