# CForest Parallelization Framework

CForest was proposed by M.Otte and N. Correll in [this paper.](http://www.mit.edu/~ottemw/html_stuff/pdf_files/otte_ieeetro2013.pdf)

The main idea behind CForest is that many trees are built in parallel between the same start and goal states. Each time a better solution is found, it is shared to all other trees. The three main characteristics of CForest are:

- Trees are expanded into regions that are known to be beneficial. Samples that cannot lead to a better solution are immediately discarded.
- Trees are pruned every time a better solution is found. Those states in the tree that do not help to find a better solution are removed from the tree.

CForest is designed to be used with any random tree algorithm under the following assumptions:

1. The search tree has almost sure convergence to the optimal solution.
2. The configuration space obeys the [triangle inequality.](http://en.wikipedia.org/wiki/Triangle_inequality) That is, there exists an admissible heuristic.

## CForest in OMPL

CForest has been included into OMPL as a new [optimizing planner](optimalPlanning.html):

- `ompl::geometric::CForest`

Currently, CForest is implemented with RRT* (ompl::geometric::RRTstar) as underlying planner, since it is the only single-query, incremental, optimal planning algorithm available.

he CForest planner is the responsible of coordinating the different trees and sharing the solutions found. Path sharing is done through a specific CForest state sampler (ompl::base::CForestStateSampler). This sampler allows to add states to sample: if a state has been added to the sampler, it will _sample_ it in the following call to \c sampleUniform() (or any of the other sampling calls).

From the user perspective, CForest can be used is any other planning algorithm:

~~~{.cpp}
// Setting up the planning problem with SimpleSetup
SimpleSetup ss;
...
ompl::base::PlannerPtr planner(new ompl::geometric::CForest(ss.getSpaceInformation()));
ss.setPlanner(planner);
...
~~~

By default, it will use 2 instances of RRT*. The number of instances and the underlying planner can be modified calling the \c setPlannerInstances<T>() method:

~~~{.cpp}
ompl::base::PlannerPtr planner(new ompl::geometric::CForest(ss.getSpaceInformation()));
// Setting up CForest to expand 5 trees of RRTstar.
planner->as<ompl::geometric::CForest>()->setPlannerInstances<ompl::geometric::RRTstar>(5);
~~~

The call \c setPlannerInstances<T>() will check the planner type given with the \c activateCForest() method. All planners that support CForest should have this method implemented. Otherwise, the following error will appear during compilation (test with PRM*):

~~~{.cpp}
planner->as<ompl::geometric::CForest>()->setPlannerInstances<ompl::geometric::PRMstar>(2);
~~~
    /ompl/src/ompl/geometric/planners/cforest/CForest.h:76:21: error: ‘class ompl::geometric::PRMstar’ has no member named ‘activateCForest’
                     planner->as<T>()->activateCForest();
                     ^

\note No Python bindings are available at the moment for this planner due to its multithreaded implementation.

#### Main differences with the paper version
When implementing CForest, the focus was modify the underlying planner as less as possible. Although the main idea of CForest remains, the actual implementation differs from the one proposed in the paper:

- No message passing is used. But shared memory and boost::threads are employed.
- The paper creates two different versions: sequential (many trees expanding in the same CPU) and parallel (1 tree per CPU). Since boost::threads are used, the trees/CPU division is done by the scheduler.
- Originally, shared states are treated in a slightly different way than randomly sampled states. Due to the CForestStateSampler encapsulation, all states are treated the same way. This can imply that shared states are not included in other trees. However, tests showed that this does not imply a high impact on performance.
- Sampling bounds are not explicitly set. Samples are created and then checked to know if they can lead to a better solution using the established heuristic.
- Start and goal states are not included in the shared paths in order to keep code simpler.
- Before pruning a tree, it is checked how many states would be removed. If the ratio size new tree/size old tree is not small enough, pruning will not be carried out. This allows to save time since the creation of a NearestNeighbors datastructure is time consuming and sometimes, just a few states are being removed.

#### Results
Note that CForest does not imply to find a first solution faster than the underlying planner used. However, it guarantees a probabilistic speed up in the optimization convergence.

The following images show the performance of CForest, using 2 RRTstar threads, in comparison with the standard RRTstar, solving the Alpha 1.5 puzzle.

\htmlonly
<div class="row">
<div class="col-md-6 col-sm-6">
  <img src="../images/cforest_cost.png" width="100%"><br>
<b>Best cost evolution through time</b> CForest converges faster towards the minimum cost (optimal solution).
</div>
<div class="col-md-6 col-sm-6">
  <img src="../images/cforest_paths.png" width="100%"><br>
<b>Number of paths shared by CForest.</b>
</div>
</div>
\endhtmlonly


#### Limitations
- Path length opt obj

- Pruning does not delete states.

#### Make your planner CForest-compatible