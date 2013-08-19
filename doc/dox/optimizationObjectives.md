# Optimization Objectives

In this tutorial, we'll discuss how to implement your own customized `OptimizationObjective`s for optimal planning. We'll continue using the previous tutorial's example planning problem with the 2D robot and circular obstacle.

## Specifying a new objective: path clearance

Previously, we considered optimal planning in terms of minimizing the length of the path found. However, this path tends to steer very close to obstacles, which can sometimes be unsafe. For safety reasons, let's define an objective which attempts to steer the robot away from obstacles. For this example, we chose to represent our metric of a path's clearance from obstacles as a summation of state costs along the path, where each state cost is a function of the state's clearance from obstacles.

~~~{.cpp}
class ClearanceObjective : public ob::StateCostIntegralObjective
{
public:
    ClearanceObjective(const ob::SpaceInformationPtr& si) :
        ob::StateCostIntegralObjective(si)
    {
        this->enableMotionCostInterpolation();
    }

    ob::Cost stateCost(const ob::State* s) const
    {
        return ob::Cost(1 / si_->getStateValidityChecker()->clearance(s));
    }
};
~~~

In the above code fragment, you'll see that we've defined our path clearance objective as a subclass of `ompl::base::StateCostIntegralObjective`. This is because `StateCostIntegralObjective` represents objectives as summations of state costs, which is exactly what we require. Therefore, all we need to do to completely specify our path clearance objective is to inherit from `StateCostIntegralObjective` and specify our state cost function by overriding the stateCost() method.

Now, let's talk about the counterintuitive implementation of stateCost(). By default, optimization objectives seek to _minimize_ path cost. For our path clearance objective, we want paths to _maximize_ path clearance. Therefore, we want our state cost function to return _smaller_ costs when states have _greater_ clearance from obstacles. An easy way to do this is to simply define the state cost as the reciprocal of that state's clearance.

Lastly, you'll notice that in `ClearanceObjective`'s constructor we called the method `enableMotionCostInterpolation()`, which is defined in `StateCostIntegralObjective`. This method changes the behaviour of how `StateCostIntegralObjective` sums up the state costs along the path. By default, `StateCostIntegralObjective` simply takes the individual states that make up a given path, and sums up those costs. However, this approach can result in an inaccurate estimation of the path cost if successive states on the path are far apart. If we enable motion cost interpolation with `enableMotionCostInterpolation()`, the path cost computation will interpolate between distant states in order to get a more accurate approximation of the true path cost. This interpolation of states along a path is the same as the one used in `ompl::base::DiscreteMotionValidator`. Note that the increase in accuracy by using motion cost interpolation comes with a decrease in computational effiency due to more calls to stateCost().

## Multiobjective optimal planning

In some cases you might be interested in optimal planning under more than one objective. For instance, we might want to specify some balance between path clearance and path length. We can do this using the `ompl::base::MultiOptimizationObjective` class.

~~~{.cpp}
ob::OptimizationObjectivePtr getBalancedObjective(const ob::SpaceInformationPtr& si)
{
    ob::OptimizationObjectivePtr lengthObj(new ob::PathLengthOptimizationObjective(si));
    ob::OptimizationObjectivePtr clearObj(new ClearanceObjective(si));

    ob::MultiOptimizationObjective* opt = new ob::MultiOptimizationObjective(si);
    opt->addObjective(lengthObj, 5.0);
    opt->addObjective(clearObj);

    return ob::OptimizationObjectivePtr(opt);
}
~~~

The above code fragment creates and optimization objective which attempts to optimize both path length and clearance. We begin by defining each of the individual objectives, and then we add them to a `MultiOptimizationObjective` object. This results in an optimization objective where path cost is equivalent to summing up each of the individual objectives' path costs. When we add objectives to `MultiOptimizationObjective`, we can also optionally specify each objective's weighting factor to signify how important it is in optimal planning. If no weight is specified, the weight defaults to 1.0. In the above example, we weigh the length with a factor of 5.0 to try to balance more in favor of minimizing path length in planning. This objective results in a path which still maintains clearance from the circle, but not as much as before.

We also provide another, more concise way to define multiple optimization objectives using operator overloading:

~~~{.cpp}
ob::OptimizationObjectivePtr getBalancedObjective(const ob::SpaceInformationPtr& si)
{
    ob::OptimizationObjectivePtr lengthObj(new ob::PathLengthOptimizationObjective(si));
    ob::OptimizationObjectivePtr clearObj(new ClearanceObjective(si));

    return 5.0*lengthObj + clearObj;
}
~~~

This function defines exactly the same optimization objective as the previous one, but uses fewer lines of code and represents the objective in a semantically rich form.
