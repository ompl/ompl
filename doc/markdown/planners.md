# Available Planners

All implementations listed below are considered fully functional. Some of the newer planners are marked as *experimental*. An experimental planner is one that, despite passing our test suite, has not been used very often and issues may still exist (bugs or efficiency issues).

# Planning under geometric constraints

This set of planners only accounts for the geometric constraints of the system.  Infinite accelerations and velocities are assumed.

- [Kinematic Planning by Interior-Exterior Cell Exploration (KPIECE)](\ref gKPIECE1)
- [Bi-directrional KPIECE (BKPIECE)](\ref gBKPIECE1)
- [Lazy Bi-directional KPIECE (LBKPIECE)](\ref gLBKPIECE1)
.
- [Single-query Bi-directional Lazy collision checking planner (SBL)](\ref gSBL)
- [Parallel Single-query Bi-directional Lazy collision checking planner (pSBL)](\ref gpSBL) \[__experimental__\]
.
- [Expansive Space Trees (EST)](\ref gEST)
- [Search Tree with Resolution Independent Density Estimation (STRIDE)](\ref gSTRIDE) \[__experimental__\]
.
- [Path-Directed Subdivision Trees (PDST)](\ref gPDST)
.
- [Rapidly-exploring Random Trees (RRT)](\ref gRRT)
- [RRT Connect (RRTConnect)](\ref gRRTC)
- [Parallel RRT (pRRT)](\ref gpRRT) \[__experimental__\]
- [Lazy RRT (LazyRRT)](\ref gLazyRRT) This planner is not experimental, but in our experience it does not seem to outperform other planners by a significant margin on any class of problems.
.
- [Probabilistic RoadMaps (PRM)](\ref gPRM)
- [PRM*](\ref gPRMstar)
- [LazyPRM](\ref gLazyPRM)
- [SPARS](\ref gSPARS)
- [SPARS2](\ref gSPARStwo)
.
- [RRT*](\ref gRRTstar)
- [Lower Bound Tree RRT (LBTRRT)](\ref gLBTRRT) \[__experimental__\]
.
- [Transition-based RRT (T-RRT)](\ref gTRRT)

Other tools:

- [Hill Climbing](\ref HillClimbing)
- [Genetic Search](\ref GeneticSearch)


# Planning under differential constraints

- [Kinodynamic Planning by Interior-Exterior Cell Exploration (KPIECE)](\ref cKPIECE1)
- [Rapidly-exploring Random Trees (RRT)](\ref cRRT)
- [Expansive Space Trees (EST)](\ref cEST)
- [Path-Directed Subdivision Trees (PDST)](\ref cPDST)
- [Syclop, a meta planner that uses other planners at a lower level](\ref cSyclop):
   - [Syclop using RRT as the low-level planner](\ref cSyclopRRT)
   - [Syclop using EST as the low-level planner](\ref cSyclopEST)

To see how to benchmark planners, click [here](benchmark.html).
