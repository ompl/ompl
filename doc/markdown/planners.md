# Available Planners {#planners}

All implementations listed below are considered fully functional. Within OMPL planners are divided into two categories:
- \ref geometric_planners
- \ref control_planners
.
To see how to benchmark planners, click [here](benchmark.html).

## Geometric planners {#geometric_planners}

Planners in this category only accounts for the geometric and kinematic constraints of the system. It is assumed that any feasible path can be turned into a dynamically feasible trajectory. Any of these planners can be used to [plan with geometric constraints](constrainedPlanning.html). Planners in this category can be divided into several overlapping subcategories:

<div class="plannerlist">
- **Multi-query planners**<br>
  These planners build a roadmap of the entire environment that can be used for multiple queries.
  - [Probabilistic Roadmap Method (PRM)](\ref gPRM)<br>
    This is the sampling-based algorithm. Our implementation uses one thread to construct a roadmap while a second thread checks whether a path exists in the roadmap between a start and goal state. OMPL contains a number of variants of PRM:
    - [LazyPRM](\ref gLazyPRM)<br>
      This planner is similar to regular PRM, but checks the validity of a vertex or edge “lazily,” i.e., only when it is part of a candidate solution path.
    - [PRM*](\ref gPRMstar)<br>
      While regular PRM attempts to connect states to a fixed number of neighbors, PRM* gradually increases the number of connection attempts as the roadmap grows in a way that provides convergence to the optimal path.
    - [LazyPRM*](\ref gLazyPRMstar)<br>
      A version of PRM* with lazy state validity checking.
  - [SPArse Roadmap Spanner algorithm (SPARS)](\ref gSPARS)<br>
    SPARS is a planner that provides asymptotic _near_-optimality (a solution that is within a constant factor of the optimal solution) and includes a meaningful stopping criterion. Although (because?) it does not guarantee optimality, its convergence rate tends to be much higher than PRM*.
  - [SPARS2](\ref gSPARStwo)<br>
    SPARS2 is variant of the SPARS algorithm that works through similar mechanics, but uses a different approach to identifying interfaces and computing shortest paths through said interfaces.
- **Single-query planners**<br>
  These planners typically grow a tree of states connected by valid motions. These planners differ in the heuristics they use to control _where_ and _how_ the tree is expanded. Some tree-based planners grow _two_ trees: one from the start and one from the goal. Such planners will attempt to connect a state in the start tree with another state in the goal tree.
  - [Rapidly-exploring Random Trees (RRT)](\ref gRRT)<br>
    This is one of the first single query planners. The algorithm is easy to understand and easy to implement. Many, many variants of RRT have been proposed. OMPL contains several RRT variants:
    - [RRT Connect (RRTConnect)](\ref gRRTC)<br>
      This planner is a bidirectional version of RRT (i.e., it grows two trees). It usually outperforms the original RRT algorithm.
    - [RRT*](\ref gRRTstar)<br>
      An asymptotically optimal version of RRT: the algorithm converges on the optimal path as a function of time. This was the first provably asymptotically planner (together with PRM). Since its publication, several other algorithms have appeared that improve on RRT*'s convergence rate, such as [RRT#](\ref gRRTsharp) and [RRTX](\ref gRRTXstatic).
    - [Lower Bound Tree RRT (LBTRRT)](\ref gLBTRRT)<br>
      LBTRRT is a asymptotically near-optimal version of RRT: it is guaranteed to converge to a solution that is within a constant factor of the optimal solution.
    - [Sparse Stable RRT](\ref gSST)<br>
      SST is an asymptotically near-optimal incremental version of RRT.
    - [Transition-based RRT (T-RRT)](\ref gTRRT)<br>
      T-RRT does not give any hard optimality guarantees, but tries to find short, low-cost paths.
    - [Vector Field RRT](\ref gVFRRT)<br>
      VF-RRT is a tree-based motion planner that tries to minimize the so-called upstream cost of a path. The upstream cost is defined by an integral over a user-defined vector field.
    - [Parallel RRT (pRRT)](\ref gpRRT)<br>
      Many different parallelization schemes have been proposed for sampling-based planners, including RRT. In this implementation, several threads simultaneously add states to the same tree. Once a solution is found, all threads terminate.
    - [Lazy RRT (LazyRRT)](\ref gLazyRRT)<br>
      This planner performs lazy state validity checking (similar to LazyPRM). It is not experimental, but in our experience it does not seem to outperform other planners by a significant margin on any class of problems.
    - [Task-space RRT (TSRRT)](\ref gTSRRT)<br>
      TSRRT is a variant of RRT where exploration is guided by the task space. It requires an ompl::geometric::TaskSpaceConfig instance that defines how to project configuration space states to the task spaces and an inverse operation to lift task space states to the configuration space.
  - [Expansive Space Trees (EST)](\ref gEST)<br>
    This planner was published around the same time as RRT. In our experience it is not as sensitive to having a good distance measure, which can be difficult to define for complex high-dimensional state spaces. There are actually three versions of EST: the [original version](\ref gEST) that is close to the first publication, [a bidirectional version](\ref gBiEST), and a [projection-based version](\ref gProjEST). The low-dimensional projection is used to keep track of how the state space has been explored. Most of the time OMPL can automatically determine a reasonable projection. We have implemented a few planners that not necessarily simple variants of EST, but do share the same expansion strategy:
    - [Single-query Bi-directional Lazy collision checking planner (SBL)](\ref gSBL)<br>
      This planner is essentially a bidirectional version of EST with lazy state validity checking.
    - [Parallel Single-query Bi-directional Lazy collision checking planner (pSBL)](\ref gpSBL)<br>
      This planner grows the two trees in SBL with multiple threads in parallel.
  - [Kinematic Planning by Interior-Exterior Cell Exploration (KPIECE)](\ref gKPIECE1)<br>
    KPIECE is a tree-based planner that uses a discretization (multiple levels, in general) to guide the exploration of the (continuous) state space. OMPL's implementation is a simplified one, using a single level of discretization: one grid. The grid is imposed on a _projection_ of the state space. When exploring the space, preference is given to the boundary of
    that part of the grid that has been explored so far. The boundary is defined to be the set of grid
    cells that have fewer than 2<i>n</i> non-diagonal non-empty neighboring grid cells in an
    _n_-dimensional projection space. There are two variants of KPIECE:
    - [Bi-directional KPIECE (BKPIECE)](\ref gBKPIECE1)
    - [Lazy Bi-directional KPIECE (LBKPIECE)](\ref gLBKPIECE1)
  - [Search Tree with Resolution Independent Density Estimation (STRIDE)](\ref gSTRIDE)<br>
    This planner was inspired by EST. Instead of using a projection, STRIDE uses a [Geometric Near-neighbor Access Tree](\ref ompl::NearestNeighborsGNAT) to estimate sampling density directly in the state space. STRIDE is a useful for high-dimensional systems where the free space cannot easily be captured with a low-dimensional (linear) projection.
  - [Path-Directed Subdivision Trees (PDST)](\ref gPDST)<br>
    PDST is a planner that has entirely removed the dependency on a distance measure, which is useful in cases where a good distance metric is hard to define. PDST maintains a binary space partitioning such that motions are completely contained within one cell of the partition. The density of motions per cell is used to guide expansion of the tree.
  - [Fast Marching Tree algorithm (FMT∗)](\ref gFMT)<br>
    The FMT∗ algorithm performs a “lazy” dynamic programming recursion on a set of probabilistically-drawn samples to grow a tree of paths, which moves outward in cost-to-come space. Unlike all other planners, the numbers of valid samples needs to be chosen beforehand.
  - [Bidirectional Fast Marching Tree algorithm (BFMT∗)](\ref gBFMT)<br>
    Executes two FMT* trees, one from the start and another one from the goal resulting in a faster planner as it explores less space.
  - [Quotient-Space RRT (QRRT)](\ref QRRT)<br>
    A generalization of RRT to plan on different abstraction levels. The abstraction levels are represented by quotient-spaces, and QRRT grows random trees sequentially and simultaneously on each quotient-space. There is extensive documentation in the form of a [guide](quotientSpacePlanning.html), [tutorial](quotientSpacePlanningTutorial.html) and [demos](group__demos.html).
- **Optimizing planners**<br>
  In recent years several sampling-based planning algorithms have been proposed that still provide some optimality guarantees. Typically, an optimal solution is assumed to be shortest path. In OMPL we have a more general framework for expressing the cost of states and paths that allows you to, e.g., maximize the minimum clearance along a path, minimize the mechanical work, or some arbitrary user-defined optimization criterion. See \ref optimalPlanning for more information. Some of the planners below use this general cost framework, but keep in mind that convergence to optimality is **not guaranteed** when optimizing over something other than path length.
  - [PRM*](\ref gPRMstar)<br> An asymptotically optimal version of PRM; _uses the general cost framework._
  - [LazyPRM*](\ref gLazyPRMstar)<br> Lazy version of PRM*; _uses the general cost framework._
  - [RRT*](\ref gRRTstar)<br> An asymptotically optimal version of RRT; _uses the general cost framework._
  - [RRT#](\ref gRRTsharp)<br> A variant of RRT* with an improved convergence rate. _It uses the general cost framework._
  - [RRTX](\ref gRRTXstatic)<br> A variant of RRT* with an improved convergence rate. _It uses the general cost framework._
  - [Informed RRT*](\ref gInformedRRTstar)<br> A variant of RRT* that uses heuristics to bound the search for optimal solutions. _It uses the general cost framework._
  - [Batch Informed Trees (BIT*)](\ref gBITstar)<br> An anytime asymptotically optimal algorithm that uses heuristics to order and bound the search for optimal solutions. _It uses the general cost framework._
  - [Advanced BIT* (ABIT*)](\ref gABITstar)<br> An extension to BIT* that uses advanced graph-search techniques to find initial solutions faster. _It uses the general cost framework._
  - [Adaptively Informed Trees (AIT*)](\ref gAITstar)<br> An anytime asymptotically optimal algorithm that simultaneously estimates and exploits problem-specific heuristics. _It uses the general cost framework._
  - [Effort Informed Trees (EIT*)](\ref gEITstar)<br> An anytime asymptotically optimal algorithm that leverages multiple sources of information to improve performance. _It uses the general cost framework._
  - [Lower Bound Tree RRT (LBTRRT)](\ref gLBTRRT)<br> An asymptotically near-optimal version of RRT.
  - [Sparse Stable RRT](\ref gSST)<br> SST is an asymptotically near-optimal incremental version of RRT.
  - [Transition-based RRT (T-RRT)](\ref gTRRT)<br> T-RRT does not give any hard optimality guarantees, but tries to find short, low-cost paths. _It uses the general cost framework._
  - [SPARS](\ref gSPARS)<br> An asymptotically near-optimal roadmap-based planner.
  - [SPARS2](\ref gSPARStwo)<br> An asymptotically near-optimal roadmap-based planner.
  - [FMT*](\ref gFMT)<br> An asymptotically optimal tree-based planner.
  - [CForest](\ref gCForest)<br> A meta-planner that runs several instances of asymptotically optimal planners in different threads. When one thread finds a better solution path, the states along the path are passed on to the other threads.
  - [AnytimePathShortening (APS)](\ref gAPS)<br> APS is a generic wrapper around one or more geometric motion planners that repeatedly applies [shortcutting](\ref ompl::geometric::PathSimplifier) and [hybridization](\ref ompl::geometric::PathHybridization) to a set of solution paths. Any number and combination of planners can be specified, each is run in a separate thread.
.

\attention How OMPL selects a geometric planner<br>
If you use the ompl::geometric::SimpleSetup class (highly recommended) to define and solve your motion planning problem, then OMPL will automatically select an appropriate planner (unless you have explicitly specified one). If the state space has a default projection (which is going to be the case if you use any of the built-in state spaces), then it will use [LBKPIECE](\ref gLBKPIECE1) if a bidirectional planner can be used and otherwise it will use [KPIECE](\ref gKPIECE1). These planners have been shown to work well consistently across many real-world motion planning problems, which is why these planners are the default choice. In case the state space has no default projection, [RRTConnect](\ref gRRTC) or regular [RRT](\ref gRRT) will be used, depending on whether a bidirectional planner can be used. The notion of a goal is very general in OMPL: it may not even be possible to sample a state that satisfies the goal, in which case OMPL cannot grow a second tree at a goal state.

\attention Feasibility and Optimality<br>
The line between feasible and optimal planners is not so black and white in practice. The line between them can be blurred with an appropriate ompl::base::PlannerTerminationCondition. For instance, the ompl::base::exactSolnPlannerTerminationCondition function returns a termination condition that causes optimizing planners to terminate once the first solution is found. As another example, using a ompl::base::CostConvergenceTerminationCondition with parameters `solutionsWindow=10` and `epsilon=1`, causes an optimizing planner to terminate when _exactly_ 10 solutions have been found. You can use ompl::base::plannerOrTerminationCondition and ompl::base::plannerAndTerminationCondition to combine planner termination conditions (e.g., “terminate when either 10 solutions have been found **or** a time limit of 10 seconds is exceeded”). See \ref plannerTerminationConditions for details.

</div>
<div>

### Other tools:

- [Hill Climbing](\ref HillClimbing)
- [Genetic Search](\ref GeneticSearch)
  
</div>

## Control-based planners {#control_planners}

<div class="plannerlist">
If the system under consideration is subject to differential constraints, then a control-based planner is used. These planners rely on [state propagation](\ref ompl::control::StatePropagator) rather than simple interpolation to generate motions. These planners do not require [a steering function](\ref ompl::control::StatePropagator::steer), but all of them (except KPIECE) will use it if the user implements it. The first two planners below are kinodynamic adaptations of the corresponding geometric planners above.
- [Rapidly-exploring Random Trees (RRT)](\ref cRRT)
- [Sparse Stable RRT](\ref cSST)<br> SST is an asymptotically near-optimal incremental version of RRT.
- [Expansive Space Trees (EST)](\ref cEST)
- [Kinodynamic Planning by Interior-Exterior Cell Exploration (KPIECE)](\ref cKPIECE1)<br>
  As the name suggest, the control-based version of KPIECE came first, and the geometric versions were derived from it.
- [Path-Directed Subdivision Trees (PDST)](\ref cPDST)<br>
  The control-based version of PDST actually came before the geometric version. Given the control-based version it was straightforward to also implement a geometric version.
- [Syclop, a meta planner that uses other planners at a lower level](\ref cSyclop)<br>
  Syclop is a meta-planner that combines a high-level guide computed over a decomposition of the state space with a low-level planning algorithm. The progress that the low-level planner makes is fed back to the high-level planner which uses this information to update the guide. There are two different versions of Syclop:
   - [Syclop using RRT as the low-level planner](\ref cSyclopRRT)
   - [Syclop using EST as the low-level planner](\ref cSyclopEST)
- [Linear Temporal Logical Planner (LTLPlanner)](\ref cLTLPlanner)<br>
  LTLPlanner finds solutions for motion planning problems where the goal is specified by a Linear Temporal Logic (LTL) specification.

\attention How OMPL selects a control-based planner<br>
If you use the ompl::control::SimpleSetup class (highly recommended) to define and solve your motion planning problem, then OMPL will automatically select an appropriate planner (unless you have explicitly specified one). If the state space has a default projection (which is going to be the case if you use any of the built-in state spaces), then it will use [KPIECE](\ref cKPIECE1). This planner has been shown to work well consistently across many real-world motion planning problems, which is why it is the default choice. In case the state space has no default projection, [RRT](\ref cRRT) will be used. Note that there are no bidirectional control-based planners, since we do not assume that there is a steering function that can connect two states _exactly_.

</div>
