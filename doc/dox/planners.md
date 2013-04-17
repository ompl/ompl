# Available Planners

# Planning under geometric constraints

This set of planners only accounts for the geometric constraints of the system.  Infinite accelerations and velocities are assumed.

- [Kinematic Planning by Interior-Exterior Cell Exploration (KPIECE)](\ref gKPIECE1)
- [Bi-directrional KPIECE (BKPIECE)](\ref gBKPIECE1)
- [Lazy Bi-directional KPIECE (LBKPIECE)](\ref gLBKPIECE1)

- [Single-query Bi-directional Lazy collision checking planner (SBL)](\ref gSBL)
- [Parallel Single-query Bi-directional Lazy collision checking planner (pSBL)](\ref gpSBL)

- [Expansive Space Trees (EST)](\ref gEST)

- [Rapidly-exploring Random Trees (RRT)](\ref gRRT)
- [RRT Connect (RRTConnect)](\ref gRRTC)
- [Parallel RRT (pRRT)](\ref gpRRT)
- [Lazy RRT (LazyRRT)](\ref gLazyRRT)

- [Probabilistic RoadMaps (PRM)](\ref gPRM)
- [PRM*](\ref gPRM)

- [RRT*](\ref gRRTstar)
- [Ball Tree RRT*](\ref gBallTreeRRTstar)

- [Transition-based RRT (T-RRT)](\ref gTRRT)

Other tools:

- [Inverse Kinematics with Hill Climbing](\ref HCIK)
- [Inverse Kinematics with Genetic Algorithms](\ref GAIK)


# Planning under differential constraints

- [Kinodynamic Planning by Interior-Exterior Cell Exploration (KPIECE)](\ref cKPIECE1)
- [Rapidly-exploring Random Trees (RRT)](\ref cRRT)
- [Expansive Space Trees (EST)](\ref cEST)
- [Path-Directed Subdivision Trees (PDST)](\ref cPDST)
- [Syclop, a meta planner that uses other planners at a lower level](\ref cSyclop):
   - [Syclop using RRT as the low-level planner](\ref cSyclopRRT)
   - [Syclop using EST as the low-level planner](\ref cSyclopEST)

To see how to benchmark planners, click [here](benchmark.html).
