# Planner Termination Conditions {#plannerTerminationConditions}

OMPL uses the ompl::base::PlannerTerminationCondition concept to let planning algorithms know that an attempt to solve a motion planning problem should be terminated. This is done by passing an instance of an ompl::base::PlannerTerminationCondition to a planner's solve method. A PlannerTerminationCondition instance is essentially a functor that can be called with zero arguments that returns `true` when the planner should terminate. For convenience, there is also a cast-to-bool operator defined so the following two if statements are equivalent:

~~~{.cpp}
// ptc is an instance of ompl::base::PlannerTerminationCondition
if (ptc()) return;
if (ptc) return;
~~~

OMPL implements several different conditions that you can use:

- `timedPlannerTerminationCondition(duration)`: a **function** that returns a PlannerTerminationCondition instance that causes the planner to terminate after a fixed amount of time has been exceeded.
- `timedPlannerTerminationCondition(duration, interval)`: a **function** that returns a PlannerTerminationCondition instance that causes the planner to terminate after a fixed amount of time has been exceeded. Unlike the previous function, this one spawns a separate thread where the time passed is checked every `interval` seconds. 
- `IterationTerminationCondition(numIterations)`: a PlannerTerminationCondition-derived **class** that causes a planner to terminate after its `eval()` method has been called `numIterations` times.
- `CostConvergenceTerminationCondition(pdef, solutionsWindow, convergenceThreshold)`: a PlannerTerminationCondition-derived **class** that causes an *optimizing* planner  to terminate once the cost for the best path has converged, which is defined as the last best cost being within a small fraction of the cumulative moving average of the last _n_ reported solutions.
- `exactSolnPlannerTerminationCondition()`: a **function** that returns a PlannerTerminationCondition instance that causes the planner to terminate after an exact (i.e., not an approximate, but possibly sub-optimal) solution has been found.
- `plannerNonTerminatingCondition()`: a **function** that returns a PlannerTerminationCondition instance that causes the planner to never terminate.
- `plannerAlwaysTerminatingCondition()`: a **function** that returns a PlannerTerminationCondition instance that causes the planner to immediately terminate.
- `plannerOrTerminationCondition(c1, c2)`: a **function** that, given two PlannerTerminationCondition instances c1 and c2, returns a PlannerTerminationCondition instance that causes the planner to terminate when either returns `true`.
- `plannerAndTerminationCondition(c1, c2)`: a **function** that, given two PlannerTerminationCondition instances c1 and c2, returns a PlannerTerminationCondition instance that causes the planner to terminate when both return `true`.

If the MORSE extension is enabled, the `ompl::base::MorseTerminationCondition` is also available, which is a PlannerTerminationCondition-derived **class** that causes a planner to terminate if the user shuts down the MORSE simulation.
