# Optimization Objectives

In this tutorial, we'll discuss how to implement your own customized `OptimizationObjective`s for optimal planning. We'll continue using the previous tutorial's example planning problem with the 2D robot and circular obstacle.

## Specifying a new objective (part 1): path clearance

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

## Specifying a new objective (part 2): maximize minimum clearance

Now we'll implement an objective which will require a lot more tinkering with the rest of the methods in `OptimizationObjective`. This objective attemps to _maximize the minimum path clearance_; that is, the cost of a given path is only a function of the closest distance between the path and an obstacle.

Here's the interface of our new objective, `MaximizeMinClearance`:
~~~{.cpp}
class MaximizeMinClearance : public OptimizationObjective
{
public:
    MaximizeMinClearance(const SpaceInformationPtr &si) :
        OptimizationObjective(si) {}
    
    virtual ob::Cost stateCost(const ob::State* s) const;
    virtual bool isCostBetterThan(ob::Cost c1, ob::Cost c2) const;
    virtual ob::Cost motionCost(const ob::State *s1, const ob::State *s2) const;
    virtual ob::Cost combineCosts(ob::Cost c1, ob::Cost c2) const;
    virtual ob::Cost identityCost() const;
    virtual ob::Cost infiniteCost() const;
};
~~~

There're lots of methods here, but we're have some really cool functionality once we've finished! We'll go through the methods one by one.

Like in the previous objectives, we can reason about the cost of a path by reasoning about the costs of individual states. Let's define our state cost:

~~~{.cpp}
ob::Cost MaximizeMinClearance::stateCost(const ob::State* s) const
{
    return ob::Cost(this->si_->getStateValidityChecker()->clearance(s));
}
~~~

You'll notice that we didn't use the reciprocal of the clearance as before. This is because, in the previous case, we were constraining ourselves to considering optimal planning as a _minimization_ of path cost (this was so that we could make use of the already-implemented functionality in `StateCostIntegralObjective`). However, we can turn the objective into a _maximization_ of cost (and therefore a maximization of clearance) by overriding the `isCostBetterThan` method of `OptimizationObjective`:

~~~{.cpp}
bool MaximizeMinClearance::isCostBetterThan(ob::Cost c1, ob::Cost c2) const
{
    return c1.v > c2.v;
}
~~~

This method is how optimal planners decide whether one path is better than another. It takes two cost values, `c1` and `c2`, and returns `true` if `c1` is considered a better cost than `c2`. The objects of type `Cost` are simply wrappers for `double` values which can be accessed with `Cost::v`; so, cost `c1` is considered better than cost `c2` if `c1`'s value (path clearance) is greater than that of `c2`.

> Note: You might be wondering why we took the trouble of wrapping `double` values in a class to represent costs (instead of using a `typedef` for instance). The reason why is _type safety_. If `Cost` were simply a `typedef` of `double`, a user might accidentally use the `<` operator instead of `isCostBetterThan`, which could cause some hard-to-find errors (this author knows from experience!). By using an object to represent costs, this mistake will be caught by the compiler.

In a way, maximizing minimum clearance can be formulated similarly to the previous objectives. Your path is a sequence of states, and the path cost can still be represented as a special combination of the state costs along the path; while in the previous cases, this combination was addition, in the case of minimum clearance, the combination is the _min_ function. We can therefore specify this functionality for our objective by overriding the `combineCosts` method:

~~~{.cpp}
ob::Cost MaximizeMinClearance::combineCosts(ob::Cost c1, ob::Cost c2) const
{
    if (c1.v < c2.v)
        return c1;
    else
        return c2;
}
~~~

The base class's default implementation of `combineCosts` simply sums the two costs given as arguments. In our case, we return the minimum of the two costs, which is equivalent to returning the minimum clearance of the two. If we accumulate the cost of a path using this operation instead of addition, we'll get the minimum clearance of the entire path as desired.

Next, we need to specify how to compute the cost of a _motion_ defined by the two endpoints of the motion. Technically, the cost of the motion comes from the minimum clearance over all states along that motion. In most real-world motion planning problems this is very difficult to compute, so we have to settle for an approximation. One approximation is to simply take the the minimum of the clearances of the two endpoints; we'll implement this approximation as an example for simplicity, but it's a much better idea to sample some interpolating states along the motion for more accuracy, as is done in `ompl::base::MinimaxObjective::motionCost`. Here's how we implement the two-endpoint approximation of motion cost:

~~~{.cpp}
ob::Cost MaximizeMinClearance::motionCost(ob::State *s1, ob::State *s2) const
{
    return this->combineCosts(this->stateCost(s1), this->stateCost(s2));
}
~~~

You'll notice that this simple approximation of motion cost can be implemented using our already-defined `combineCost` and `stateCost` methods.

Many optimal planners count on their objectives having certain cost values which have special properties. One common cost value is the _identity cost_. This is a cost value `c0` which, when combined with any other cost value `c1` using `combineCost`, always returns the value `c1`. For example, when `combineCost` is simple addition, the identity cost is 0. We can specify an objective's identity cost by overriding the `identityCost` method; but what is the identity cost of our minimum cost objective? It's a cost that, when combined with another cost with the _min_ function, always returns the other cost. This means that the identity cost must be _greater_ than every other cost - in other words, infinity!

~~~{.cpp}
ob::Cost MaximizeMinClearance::identityCost() const
{
    return ob::Cost(std::numeric_limits<double>::infinity());
}
~~~

We define the identity cost of our minimum path clearance objective to be infinity. Because we're working with costs, we wrap the `double` value of infinity in a `Cost` object.

Another cost value commonly used in optimal planners is the _infinite cost_. This is a cost which is _worse than all other cost values_; in other words, it's a value `c_i` for which `isCostBetterThan(c_i, c)` is false for all values of `c`. In the case of our minimum clearance objective, we can use the value of negative infinity to fulfill this role. We specify this by overriding the `infiniteCost()` method:

~~~{.cpp}
ob::Cost MaximizeMinClearance::infiniteCost() const
{
    return ob::Cost(-std::numeric_limits<double>::infinity());
}
~~~

Our objective is now ready to be used for planning! In the previous tutorial, we used the RRTstar planner for optimal planning; you may find you get nicer results if you use the PRMstar planner for this problem. You can use it with the following code (after defining your planning problem and optimization objective):

~~~{.cpp}
ob::PlannerPtr planner(new og::PRMstar(si));
planner->setProblemDefinition(pdef);
planner->setup();
ob::PlannerStatus solved = planner->solve(timeLimit);
~~~
