# CForest Parallelization Framework

__Contents:__

- \ref cf_ompl
 - \ref cf_diff
 - \ref cf_examples
 - \ref cf_results
 - \ref cf_advanced
  - \ref cf_implementation
  - \ref cf_limitations
  - \ref cf_compatible

CForest was proposed by M.Otte and N. Correll in [this paper.](http://www.mit.edu/~ottemw/html_stuff/pdf_files/otte_ieeetro2013.pdf)

The main idea behind CForest is that many trees are built in parallel between the same start and goal states. The key concepts of CForest are:

- Every time a tree finds a better solution, it is shared to all other trees so that all trees have the best solution found so far.
- Trees are expanded into regions that are known to be beneficial. Samples that cannot lead to a better solution are immediately discarded.
- Trees are pruned every time a better solution is found. Those states in the tree that do not help to find a better solution are removed from the tree.

CForest is designed to be used with any random tree algorithm under the following assumptions:

1. The search tree has almost sure convergence to the optimal solution.
2. The configuration space obeys the [triangle inequality.](http://en.wikipedia.org/wiki/Triangle_inequality) That is, there exists an admissible heuristic.

## CForest in OMPL {#cf_ompl}

CForest has been included into OMPL as a new [optimizing planner](optimalPlanning.html):

- `ompl::geometric::CForest`

\Note CForest is designed to optimize path lengths. In OMPL, it is implemented focused on \c PathLengthOptimizationObjective. However, other shortest-paths problems could be faced: shortest in terms of time, consumed energy (not tested). Therefore, CForest in OMPL requirements are not for the state space but for the optimization objective. It requires an optimization objective with an admissible heuristic. Since this is complex to check, a warning is shown if the state space is not a metric space (although this does not mean that the optimization objective is not valid to be used with CForest).

Currently RRT* (ompl::geometric::RRTstar) is the only underlying planner available, since it is the only single-query, incremental, optimal planning algorithm implemented in OMPL.

The CForest planner is the responsible of coordinating the different trees and sharing the solutions found. Path sharing is done through a specific CForest state sampler (ompl::base::CForestStateSampler). This sampler allows to add states to sample: if a state has been added to the sampler, it will _sample_ it in the following call to \c sampleUniform() (or any of the other sampling calls).

From the user perspective, CForest can be used as any other planning algorithm:

~~~{.cpp}
#include <ompl/geometric/planners/cforest/CForest.h>
...
// Setting up the planning problem with SimpleSetup
SimpleSetup ss;
...
ompl::base::PlannerPtr planner(new ompl::geometric::CForest(ss.getSpaceInformation()));
ss.setPlanner(planner);
...
~~~

By default, it will use as many RRT* instances as cores available. The number of instances and the underlying planner can be modified calling the \c addPlannerInstances<T>() method:

~~~{.cpp}
ompl::base::PlannerPtr planner(new ompl::geometric::CForest(ss.getSpaceInformation()));
// Setting up CForest to expand 5 trees of RRTstar.
planner->as<ompl::geometric::CForest>()->addPlannerInstances<ompl::geometric::RRTstar>(5);
~~~

The call \c addPlannerInstances<T>() will check the \c PlannerSpecs of the planner type given. CForest only supports all those planners with the \c canReportIntermediateSolutions spec true. Otherwise, the following warning will appear during execution (test with PRM*):**

~~~{.cpp}
planner->as<ompl::geometric::CForest>()->addPlannerInstances<ompl::geometric::PRMstar>(2);
~~~
    Warning: PRMstar cannot report intermediate solutions, not added as CForest planner.
             at line 100 in ompl/geometric/planners/cforest/CForest.h

If not valid planners are added by the user, two instances of RRTstar will be automatically set.

Alternatively, only the number of threads could be specified and the default underlying planner (RRT*) will be chosen. This is specially useful when using the planner with a benchmark configuration file:

~~~{.cpp}
// Using 6 threads of the default planner
planner->as<ompl::geometric::CForest>()->setNumThreads(6);
~~~

\note No Python bindings are available for this planner due to its multithreaded implementation.

### Main differences with the paper version {#cf_diff}
When implementing CForest, the focus was o modify the underlying planner as less as possible. Although the main idea of CForest remains, the actual implementation differs from the one proposed in the paper:

- No message passing is used. But shared memory and boost::threads are employed.
- The paper creates two different versions: sequential (many trees expanding in the same CPU) and parallel (1 tree per CPU). Since boost::threads are used, the trees/CPU division is done by the scheduler.
- Originally, shared states are treated in a slightly different way than randomly sampled states. Due to the CForestStateSampler encapsulation, all states are treated the same way. This can imply that shared states are not included in other trees. However, tests showed that this does not imply a high impact on performance.
- Sampling bounds are not explicitly set. Samples are created and then checked to know if they can lead to a better solution using the established heuristic.
- Start and goal states are not included in the shared paths in order to keep code simpler.
- Before pruning a tree, it is checked how many states would be removed. If the ratio size new tree/size old tree is not small enough, pruning will not be carried out. This allows to save time since the creation of a NearestNeighbors datastructure is time consuming and sometimes, just a few states are being removed.

\note Despite all these differences, the CForest implementation greatly improves the performance of the underlying  planner. However, an implementation more close to the one described in the paper should improve the performance even more (specially with message passing and not treating all the states the same way). Please, take that into account if you plan to compare your algorithm against CForest.


### Examples {#cf_examples}

- [CForest demo](CForestDemo_8cpp_source.html). It shows how CForest can be used (as any other planner) and how to configure it.
- [Circle Grid benchmark](CForestCircleGridBenchmark_8cpp_source.html). Compares the perfomance of CForest against RRT  in a specific 2D circle grid problem.

### Results {#cf_results}
Note that CForest does not imply to find a first solution faster than the underlying planner used. However, it guarantees a probabilistic speed up in the optimization convergence.

CForest produces many interesting results. All these results are obtained with the alpha 1.5 puzzle benchmark configuration included in the OMPLapp. Following figure shows the results of running CForest in 2 and 16 threads in a **16-core** machine. Also, the standard RRT* and the pruned version are included in the benchmark.

\htmlonly
<div class="row">
<div class="col-md-6 col-sm-6">
  <img src="../images/cforest.png" width="100%"><br>
<b>Best cost evolution through time</b> CForest converges faster towards the minimum cost (optimal solution) as the threads are increased. Pruned RRT* also improves the standard RRT*.
</div>
<div class="col-md-6 col-sm-6">
  <img src="../images/sharedpaths.png" width="100%"><br>
<b>Number of paths shared by CForest.</b>
</div>
</div>
\endhtmlonly

Another interesting experiment is to run CForest with pruning deactivated and compare it to the standard CForest:

\htmlonly
<div class="row">
  <img src="../images/prunevsnoprune.png" width="100%"><br>
<b>Best cost evolution through time</b> Not pruning trees affects negatively on the convergence. However, it still improves the standard RRT.
</div>
\endhtmlonly

In case you only have one core available, CForest still improves the RRTstar performance! The following figure shows that using CForest in one single core but with many threads (in the picture 4 and 8) also improves the convergence rate against standard RRT* and its pruned version, reaching lower cost solutions in much less time.

<div class="row">
  <img src="../images/threadscforest.png" width="100%"><br>
<b>Best cost evolution through time</b> Sequential version of the CForest: many threads working in the same core.
</div>
\endhtmlonly


### Advanced information {#cf_advanced}
#### Implementation details {#cf_implementation}
The CForest _planner_ comes together with its own state sampler ompl::base::CForestStateSampler and its own state space ompl::base::CForestStateSpaceWrapper. They are completely transparent to the user as the ompl::geometric::CForest handles the creation of these.

CForest operates on the user specified ompl::base::SpaceInformation. However, CForest instantiates the underlying planners with an individual SpaceInformation instance for each planner, containing an instance of a \c CForestStateSpaceWrapper. When the underlying planner allocates the \c StateSampler, \c CForestStateSpaceWrapper creates an instance of a \c CForestStateSampler which wraps the user specified state sampler (or the default sampler if none was provided).

Therefore, CForest tracks the creation of the planners but, thanks to the \c CForestStateSpace, it also tracks the creation of the state samplers as well. This allows to have a planner-sampler correspondence required to shared paths between trees.

#### Limitations {#cf_limitations}
- CForest is designed to solve single-query, shortest path planning problems. Therefore, not all the ompl::base::OptimizationObjective instantiations are valid. The cost metric have to obey the triangle inequiality. It is important to note that shortest path planning does not mean that only the path length can be optimized. Other metrics could be specified: time, energy, etc. However, clearance or smoothness optimization are examples of non-valid optimization objetives.

- Whenever the tree is pruned, the states are removed from the \c NearestNeighbours data structure. However, current implementation does not remove pruned states until the planner instance is destroyed. This is specific for the underlying planner implementations but this is the most efficient way in terms of computation time.

#### Make your planner CForest-compatible {#cf_compatible}
If you have implemented an __incremental, optimizing planner__ in OMPL and want make it complatible with CForest, there are few modifications you should carry out on your planner.

There are three main components that should be included:
- CForest activation and configuration.
- Path sharing.
- Tree pruning.


##### CForest activation and configuration
Firstly, if working under CForest, the corresponding callback for sharing intermediate solutions must be set. Also, a check should be included to figure out if the pruning would cause some errors. Therefore, in the \c solve() function, you need to add the following code:

~~~{.cpp}
ompl::geoemtric::MyPlanner solve(const base::PlannerTerminationCondition &ptc)
{
    // Configuration before entering the main solving loop.
    ....
    if (prune_ && !si_->getStateSpace()->isMetricSpace())
       OMPL_WARN("%s: tree pruning was activated but since the state space %s is not a metric space, the optimization objective might not satisfy the triangle inequality. You may need to disable pruning.", getName().c_str(), si_->getStateSpace()->getName().c_str());

    const base::ReportIntermediateSolutionFn intermediateSolutionCallback = pdef_->getIntermediateSolutionCallback();
    ...
    while (ptc == false) // Main loop
    ...
}
~~~

\c prune_ flag will be set but the user or the CForest framework. This flag manages the code related to tree pruning and early state rejection. If \c intermediateSolutionCallback is false, then the path sharing code will not be executed.

Since all CForest threads share the same \c PlannerTerminationCondition, it is recommended to include the following call once the main loop of the solve() function has finished:

~~~{.cpp}
ompl::geoemtric::MyPlanner solve(const base::PlannerTerminationCondition &ptc)
{
    while (ptc == false) // Main loop
    {
        ...
    }
    
    ptc.terminate(); // Will force other threads to stop.
}
~~~

This is particularly useful when one of the threads breaks out the loop but the ptc will still evaluate to true. For instance, this happens in RRTstar everytime a cost threshold is set.

\note CForest can be used wihtout pruning. In this case, the \c prune_ flag is activated only if the ompl::geometricCForest::setPrune() method was called with a true argument (it is activated by default). Tree pruning in RRTstar can be used as an independent feature.

##### Path sharing
As CForest is designed to use incremental, optimizing planners, it is assumed you will have a flag in your code to indicate when a new, better path has been found and a pointer to the motion which contains the last state (goal) of the solution. Therefore, at the end of the main loop within the solve function, you should add a code similar to the following:

~~~{.cpp}
ompl::geoemtric::MyPlanner solve(const base::PlannerTerminationCondition &ptc)
{
    // Configuration before entering the main solving loop.
    ...
    while (ptc == false) // Main loop
    {
        ...
        // Sampling new states.
        ...
        // Addind states to the tree.
        ...
        // Searching for improved solutions.
        ...
		if (updatedSolution)
        {
            ...
            if (intermediateSolutionCallback)
            {
                std::vector<const base::State *> spath;
                Motion *intermediate_solution = solution->parent;

                do
                {
                    spath.push_back(intermediate_solution->state);
                    intermediate_solution = intermediate_solution->parent;
                } while (intermediate_solution->parent != 0);

                intermediateSolutionCallback(this, spath, bestCost_);
            }
        } // if updated solution
    } // while ptc
}
~~~

\note The spath vector has to contain the states of the solution from the goal to the start (in this specific order).

In this case, the goal and start states are not being included since it is usually harder to deal with those two states. Code simplicity prevails, but this really depends on the underlying planner used together with CForest.

It is likely to share more than one path in which one or more states are modified. This would imply that the tree and solution path would have repeated states. To avoid this, add the following code within the \c solve() method just after sampling a new state:

~~~{.cpp}
ompl::geoemtric::MyPlanner solve(const base::PlannerTerminationCondition &ptc)
{
    // Configuration before entering the main solving loop.
    ...
    while (ptc == false) // Main loop
    {
        // Sample a new state and store it in rmotion.
        ...
        // find closest state in the tree
        Motion *nmotion = nn_->nearest(rmotion);
        if (intermediateSolutionCallback && si_->equalStates(nmotion->state, rstate))
            continue;
        ...
    }
}
~~~

If a sampled state is repeated it will be discarded and the next iteration of the main loop will start.

##### Tree pruning
Prunning refers to two different ways of remove states: 1) prune those states already on the tree that do not lead to a better solution and 2) reject those states before adding them to the tree. In both cases, all the modifications are again within the \c solve() method. Tree main modifications are done:

1. **Early state rejection** Check if random samples can lead to a better solution (cost used: heuristic from start to state + heuristic from state to goal)
2. **State rejection**Most of the states satisfy 1., but once they are wired into the tree, they cannot lead to a better solution (cost used: cost to go from start to state + heuristic from state to goal)  (early state rejection)..
3. **Tree pruning** If a new, better solution is found, the pruning threshold will be decreased, therefore prune the states of the tree so that all those states which higher cost than the current best cost are removed.

All these modifications are included in the following code example:

~~~{.cpp}
ompl::geoemtric::MyPlanner solve(const base::PlannerTerminationCondition &ptc)
{
    // Configuration before entering the main solving loop.
    ...
    while (ptc == false) // Main loop
    {
        ...
        // Sampling new states.
        ...
        sampler_->sampleUniform(rstate);

        if (prune) // Modification 1 - early state rejection
        {
            const base::Cost costTotal =  computeLowestCostToGo(rmotion);
            if (opt_->isCostBetterThan(bestCost_, costTotal))
                continue;
        }

        // Addind states to the tree.
        ...
        if (prune) // Modification 2 - state rejection.
        {
            const base::Cost costTotal = computeCostToGoal(motion);
            if (opt_->isCostBetterThan(costTotal, bestCost_))
            {
                nn_->add(motion);
                motion->parent->children.push_back(motion);
            }
            else // If the new motion does not improve the best cost it is ignored.
            {
                si_->freeState(motion->state);
                delete motion;
                continue;
            }
        }
        else // Regular code when prune is not activated.
        {
            // add motion to the tree
            nn_->add(motion);
            motion->parent->children.push_back(motion);
        }
        // Searching for improved solutions.
        ...
		if (updatedSolution)
        {
             ...
             if (prune_) // Modification 3 - tree pruning.
                 pruneTree(bestCost_);
             ...
        } // if updated solution
    } // while ptc
}
~~~

\note You should implement the \c pruneTree() function for your code. Most probably, the available RRTstar::pruneTree() method would be directly applicable. It is possible to use CForest without this method. However, it is highly recommended to at least include the modifications 1 & 2 about state rejection.

For a complete example of how to make these modifications, it is recommended to analyze the ompl::geometric::RRTstar::solve() method.