# Planner Termination Conditions {#plannerTerminationConditions}

OMPL uses the ompl::base::PlannerTerminationCondition concept to let planning algorithms know that an attempt to solve a motion planning problem should be terminated. This is done by passing an instance of an ompl::base::PlannerTerminationCondition to a planner's solve method. A PlannerTerminationCondition instance is essentially a functor that can be called with zero arguments that returns `true` when the planner should terminate. For convenience, there is also a cast-to-bool operator defined so the following two if statements are equivalent:

~~~{.cpp}
// ptc is an instance of ompl::base::PlannerTerminationCondition
if (ptc()) return;
if (ptc) return;
~~~

OMPL implements several different conditions that you can use:

- ompl::base::timedPlannerTerminationCondition(double): a **function** that returns a PlannerTerminationCondition instance that causes the planner to terminate after a fixed amount of time has been exceeded.
- ompl::base::timedPlannerTerminationCondition(ompl::time::duration): same as the previous one, but using an OMPL data type to denote the duration.
- ompl::base::timedPlannerTerminationCondition(double, double): a **function** that returns a PlannerTerminationCondition instance that causes the planner to terminate after a fixed amount of time has been exceeded. Unlike the previous function, this one spawns a separate thread where the time passed is checked every `interval` seconds.
- ompl::base::IterationTerminationCondition: a PlannerTerminationCondition-derived **class** that causes a planner to terminate after its `eval()` method has been called `numIterations` times, where `numIterations` is an unsigned int argument to the constructor.
- ompl::base::CostConvergenceTerminationCondition: a PlannerTerminationCondition-derived **class** that causes an *optimizing* planner to terminate once the cost for the best path has converged, which is defined as the cumulative moving average of the last _n_ reported solutions changing by a factor of less than _epsilon_. By setting _epsilon_ to 1, the planner will automatically terminate after exactly _n_ solutions have been found.
- ompl::base::exactSolnPlannerTerminationCondition: a **function** that returns a PlannerTerminationCondition instance that causes the planner to terminate after an exact (i.e., not an approximate, but possibly sub-optimal) solution has been found.
- ompl::base::plannerNonTerminatingCondition: a **function** that returns a PlannerTerminationCondition instance that causes the planner to never terminate.
- ompl::base::plannerAlwaysTerminatingCondition: a **function** that returns a PlannerTerminationCondition instance that causes the planner to immediately terminate.
- ompl::base::plannerOrTerminationCondition: a **function** that, given two PlannerTerminationCondition instances c1 and c2, returns a PlannerTerminationCondition instance that causes the planner to terminate when either returns `true`.
- ompl::base::plannerAndTerminationCondition: a **function** that, given two PlannerTerminationCondition instances c1 and c2, returns a PlannerTerminationCondition instance that causes the planner to terminate when both return `true`.

If the MORSE extension is enabled, the ompl::base::MorseTerminationCondition is also available, which is a PlannerTerminationCondition-derived **class** that causes a planner to terminate if the user shuts down the MORSE simulation.
