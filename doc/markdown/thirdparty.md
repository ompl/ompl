# Contributions {#thirdparty}

This page includes a list of extensions to OMPL provided by the community. Please read the [directions on writing a contribution](contrib.html) and the suggested [style guide](styleGuide.html) if you would like to submit your own contribution.

## CForest (Coupled Forest of Random Engrafting Search Trees)

### Author: Javier V Gomez (Universidad Carlos III de Madrid)

CForest is a parallelization framework that is designed for single-query shortest
path planning algorithms. It is not a planning algorithm _per se_.

CForest is designed to be used with any random tree algorithm that operates
in any configuration space such that: 1) the search tree has almost sure
convergence to the optimal solution and 2) the configuration space obeys
the triangle inequality. It relies in the OptimizationObjective set for
the underlying planners.

See also the extensive documentation [here](CForest.html). This also describes how our implementation differs from the algorithm as described in:

- M. Otte, N. Correll, C-FOREST: Parallel Shortest Path Planning With
Superlinear Speedup, IEEE Transactions on Robotics, Vol 20, No 3, 2013.
DOI: [10.1109/TRO.2013.2240176](https://dx.doi.org/10.1109/TRO.2013.2240176)  
[[PDF]](https://dx.doi.org/10.1109/TRO.2013.2240176)

## Fast Marching Tree algorithm (FMT∗)

### Authors: Ashley Clark and Wolfgang Pointner (Stanford University)

[FMT∗](\ref gFMT) is a new asymptotically optimal algorithm contributed by Marco Pavone's Autonomous Systems Laboratory. The FMT∗ algorithm performs a “lazy” dynamic programming recursion on a set of probabilistically-drawn samples to grow a tree of paths, which moves outward in cost-to-come space. The algorithm is described in:

- L. Janson, E. Schmerling, A. Clark, M. Pavone. Fast marching tree: a fast marching sampling-based method for optimal motion planning in many dimensions. The International Journal of Robotics Research, 34(7):883-921, 2015. DOI: [10.1177/0278364915577958](https://dx.doi.org/10.1177/0278364915577958) [[PDF]](https://arxiv.org/pdf/1306.3532.pdf)

## Bidirectional Fast Marching Tree algorithm (BFMT∗)

### Authors: Joseph Starek (Stanford University) and Javier V Gomez (Universidad Carlos III de Madrid)

[BFMT∗](\ref gBFMT) is a new asymptotically optimal algorithm contributed by Marco Pavone's Autonomous Systems Laboratory. The BFMT∗ algorithm expands two Fast Marching Trees, one from the initial state and the other from the goal, resulting in faster convergence rates are less space is explored by the trees. The algorithm is described in:

- J. A. Starek, J. V. Gomez, E. Schmerling, L. Janson, L. Moreno, and M. Pavone, An Asymptotically-Optimal Sampling-Based Algorithm for Bi-directional Motion Planning, in IEEE/RSJ International Conference on Intelligent Robots Systems, 2015. [[PDF]](https://arxiv.org/pdf/1507.07602.pdf)

## Generalized Optimal Path Planning Framework

### Author: Luis Torres (University of North Carolina at Chapel Hill)

Extend OMPL's support for optimal path planning. Users can plan robot paths that optimize a variety of ready-to-use metrics such as path length and clearance from obstacles. Users can also plan optimal paths under their own customized path metrics without changing OMPL's optimizing planners. Additional benchmarking data has also been added so users can efficiently track the progress of an optimizing planner on their planning problem. Tutorials for use of the framework can be found [here](optimalPlanning.html).

## LBT-RRT implementation

### Author: Oren Salzman (Tel Aviv University)

[LBT-RRT](\ref gLBTRRT) (Lower Bound Tree RRT) is a near-asymptotically optimal incremental sampling-based motion planning algorithm. LBT-RRT is guaranteed to converge to a solution that is within a constant factor of the optimal solution. The notion of optimality is with respect to the distance function defined on the state space we are operating on. The algorithm is described in:

- Oren Salzman and Dan Halperin, Asymptotically near-optimal RRT for fast, high-quality, motion planning,
_arXiv_, 2013 [[PDF]](https://arxiv.org/pdf/1308.0189.pdf)

## SPARS and SPARS2 implementations

### Author: Andrew Dobson (Rutgers University)

- [SPARS](\ref gSPARS) and [SPARS2](\ref gSPARStwo) are roadmap-based planners that operate similarly to Visbility-based PRM, but provide asymptotic near-optimality guarantees. The SPARS and SPARS2 algorithms a described in:

  - A. Dobson, A. Krontiris, K. Bekris, Sparse Roadmap Spanners, _Workshop on the Algorithmic Foundations of Robotics (WAFR)_ 2012. [[PDF]](https://www.cs.rutgers.edu/~kb572/pubs/sparse_roadmap_spanner.pdf)
  - A. Dobson, K. Bekris, Improving Sparse Roadmap Spanners, _IEEE International Conference on Robotics and Automation (ICRA)_ May 2013. [[PDF]](https://www.cs.rutgers.edu/~kb572/pubs/spars2.pdf)

## T-RRT implementation

### Author: Dave Coleman (University of Colorado Boulder)

- [T-RRT](\ref gTRRT) is an RRT variant and tree-based motion planner that takes into consideration state costs to compute low-cost paths that follow valleys and saddle points of the configuration-space costmap. It uses transition tests from stochastic optimization methods to accept or reject new potential sates. [An example use of TRRT](https://github.com/davetcoleman/ompl_rviz_viewer).
  - L. Jaillet, J. Cortés, T. Siméon, Sampling-Based Path Planning on Configuration-Space Costmaps, in IEEE Transactions on Robotics, 26(4):635–646, August 2010, [[PDF]](https://homepages.laas.fr/nic/Papers/10TRO.pdf)

## ROS interface to OMPL

### Authors: Ioan Sucan and Sachin Chitta (Willow Garage)

- [MoveIt!](https://moveit.ros.org) wraps OMPL as a planning plugin.

## RRT* and extensions

### Authors: Alejandro Perez and Sertac Karaman (MIT)

- [RRT*](\ref gRRTstar) (optimal [RRT](\ref gRRT)) is an asymptotically-optimal incremental sampling-based motion planning algorithm. RRT* algorithm is guaranteed to converge to an optimal solution, while its running time is guaranteed to be a constant factor of the running time of the RRT. The RRT* algorithm was introduced and analyzed in:

  - S. Karaman and E. Frazzoli, _Sampling-based algorithms for optimal motion planning_ Int. Journal of Robotics Research, 2011. Also available at <https://arxiv.org/abs/1105.1186>.

## PRM extensions

### Author: James Marble (University of Nevada at Reno)

- Generalized the implementation of [PRM](\ref gPRM) so that different variations can be created by passing connection strategies and filters. Provided such functions for PRM and [PRM*](\ref gPRMstar).

## RRT extension for planning with controls

### Author: Jennifer Barry (MIT)

- Generalized the implementation of RRT (with controls) so that intermediate states generated along motions are also optionally added to that tree of motions.

## QuotientSpaceRRT extension

### Author: Andreas Orthey (University of Stuttgart)

- [QRRT](\ref QRRT) A generalization of RRT to plan on different abstraction levels. The abstraction levels are represented by quotient-spaces, and QRRT grows random trees sequentially and simultaneously on each quotient-space.
See also our [guide](quotientSpacePlanning.html), [tutorial](quotientSpacePlanningTutorial.html) and [demos](group__demos.html).
  - A Orthey and M Toussaint, _Rapidly-Exploring Quotient-Space Trees: Motion Planning using Sequential Simplifications_, 2019. Also available at <https://arxiv.org/abs/1906.01350>.

  - A Orthey, A Escande and E Yoshida, _Quotient Space Motion Planning_, ICRA, 2018. Also available at <https://arxiv.org/abs/1807.09468>.
