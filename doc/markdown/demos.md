# Demos

\defgroup demos Demos
\ingroup demos
@{

- [Rigid body planning](RigidBodyPlanning_8cpp_source.html) [[Python version]](RigidBodyPlanning_8py_source.html). This demo is essentially the same example described in the first tutorial. It illustrates how to use the main classes.
- [State sampling](StateSampling_8cpp_source.html) [[Python version]](StateSampling_8py_source.html). This is the demo program used in the last beginner tutorial.
- [Analyze and visualize planner data in Python.](PlannerData_8py_source.html) This demo relies on the [graph-tool](http://projects.skewed.de/graph-tool) package.
- [Rigid body planning with controls](RigidBodyPlanningWithControls_8cpp_source.html) [[Python version]](RigidBodyPlanningWithControls_8py_source.html). This demo shows how to perform planning under differential constraints for a simple car-like vehicle.
- [Rigid body planning with integration and controls.](RigidBodyPlanningWithIntegrationAndControls_8cpp_source.html) This example extends the previous example by showing how one can plan for systems of ordinary differential equations in a generic way. This example uses simple Euler integration. For higher accuracy it is recommended to use the ODESolver class described in the next demo.
- [Rigid body planning with ODESolver and controls.](RigidBodyPlanningWithODESolverAndControls_8cpp_source.html)  [[Python version]](RigidBodyPlanningWithODESolverAndControls_8py_source.html) This example compares and contrasts the previous demo of planning with integration and planning using the ODESolver class, which wraps around [Boost.Numeric.Odeint](http://www.boost.org/libs/numeric/odeint).  Code showing the same model being planned with a user-implemented numerical integration technique as well as the ODESolver is presented.
- [Koules](Koules_8cpp_source.html) This is a very elaborate demo to illustrate planning for underactuated kinodynamic systems with drift. It based on an old Unix game called [Koules](http://www.ucw.cz/~hubicka/koules/English/). The physics have been made significantly harder compared to the original game. The demo can solve just one level of Koules, all levels, or run a number of planners on one level as a benchmarking run. The demo prints out a path that can be turned into a movie using KoulesPlayback.py. Example usage:

        > ./demo_Koules --planall --planner kpiece --numkoules 7 --maxtime 600 --output koules7.log
        > /path/to/ompl/demos/KoulesPlaypack.py koules7.log

  This will create a movie called koules7.mp4. See [the gallery](gallery.html) for an example of what this might look like.

  This demo illustrates also many advanced OMPL concepts, such as classes for a custom state space, a control sampler, a projection, a state propagator, and a goal class. It also demonstrates how one could put a simple bang-bang controller inside the StatePropagator. In this demo the (Directed)ControlSampler simply samples a target velocity vector and inside the StatePropagator the control is chosen to drive the ship to attain this velocity.
- [Planning for a simple hybrid system.](HybridSystemPlanning_8cpp_source.html) This demo shows how one could plan for a car with gears. The gear is a discrete state variable, while its pose is continuous. The car needs to make a sharp turn and is forced to change gears. This is not the best way to plan for hybrid systems, since this approach ignores completely the structure that exist in the system. Nevertheless, it demonstrates that the planners in OMPL are state space agnostic and can plan in discrete or hybrid state spaces.
- [Rigid body planning with an Inverse Kinematics solver generating goal states in a separate thread.](RigidBodyPlanningWithIK_8cpp_source.html) This demo shows off two neat features of OMPL: a genetic algorithm-based Inverse Kinematics solver and a lazy goal state sampler. In a separate thread goal states are computed by the IK solver. While solving a motion planning problem, the planning algorithms select a random goal state from the ones computed so far.
- [Rigid body planning using the Open Dynamics Engine (OpenDE).](OpenDERigidBodyPlanning_8cpp_source.html) When OpenDE is installed, OMPL will compile an extension that makes is easier to use OpenDE for forward propagation of models of motion. In this example, a box is pushed around in the plane from a start position to a goal position.
- [Random walk planner](RandomWalkPlanner_8py_source.html) A simple demo illustrating how to create a new planning algorithm in Python. This particular planner simply performs a random walk until it gets close to the goal.
- [Planning for Dubins and Reeds-Shepp cars.](GeometricCarPlanning_8cpp_source.html) This demo illustrates the use of the ompl::base::DubinsStateSpace and ompl::base::ReedsSheppStateSpace. The demo can solve two simple planning problems, print trajectories from the origin to a user-specified state, or print a discretized distance field.
- [Optimal planning for a 2D point robot.](OptimalPlanning_8cpp_source.html) [[Python version]](OptimalPlanning_8py_source.html). This demo illustrates the use of `ompl::base::OptimizationObjective` to construct optimization objectives for optimal motion planning.
- [Hypercube benchmark.](HypercubeBenchmark_8cpp_source.html) A simple benchmark where the configuration space consists of a hypercube in R<sup>n</sup> and the free space is a narrow corridor along edges of the hypercube. The exploration progress of a planner is therefore hard to capture in a low-dimensional projection.
- [Kinematic chain benchmark.](KinematicChainBenchmark_8cpp_source.html) A benchmark for an _n_-link kinematic chain to get out of a narrow passage. It requires the chain to fold up and expand again. As in the previous benchmark, the free space is hard to capture by a low-dimensional projection of the configuration space.
- [2D point planning using a PPM image as a map](Point2DPlanning_8cpp_source.html) [[Python version](Point2DPlanning_8py_source.html)]
- [Using the CForest framework](CForestDemo_8cpp_source.html) Modifies the same 2D point planning as before but the problem is solved with the CForest parallelization framework and compared with RRTstar.
- [Circle Grid benchmark](CForestCircleGridBenchmark_8cpp_source.html). Implements a configurable 2D circle grid benchmark problem, where the user can specify state space, size of the grid and circles, so that the problem can be as hard as desired.

@}
