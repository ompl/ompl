# Representing Goals in OMPL {#goalRepresentation}

## Setting the Goal

The most general representation of a goal is ompl::base::Goal. This class contains a pure virtual method, ompl::base::Goal::isSatisfied(), which takes a state as argument and returns a boolean indicating whether that state is a goal or not. No other information about the goal is given. This function can include arbitrary code deciding what is in the goal region and what is not.

~~~{.cpp}
class MyArbitraryGoal : public ompl::base::Goal
{
public:
    MyArbitraryGoal(const SpaceInformationPtr &si) : ompl::base::Goal(si)
    {
    }

    virtual bool isSatisfied(const State *st) const
    {
        // perform any operations and return a truth value
    }
};
~~~

While this is a very general definition, it is often the case more information is known about the goal. This information is helpful for planners and the user should specify this information when available. Here are types of information planners can use:

- If an approximation of distance to the goal is available, not necessarily a metric, this information can be given to the planner by also providing an implementation of ompl::base::Goal::isSatisfied() (the version that takes a ompl::base::State* and a double* as arguments). By default, the implementation of this function simply sets the distance to the maximum value a double can hold.

~~~{.cpp}
class MyArbitraryGoal : public ompl::base::Goal
{
public:
    MyArbitraryGoal(const SpaceInformationPtr &si) : ompl::base::Goal(si)
    {
    }

    virtual bool isSatisfied(const State *st) const
        {
        perform_any_operations();
        return truth value;
    }

    virtual bool isSatisfied(const State *st, double *distance) const
        {
        bool result = isSatisfied(st);

        if (distance != NULL)
        {
            if (state_clearly_out_of_goal_region(st))
                    {
                *distance = std::numeric_limits<double>::max();
            }
            else
            {
                if (state_near_goal(st))
                    *distance = 1;
                else
                    *distance = 100;
            }
        }
        return result;
    }
};
~~~

- It is often the case that when distance to goal approximations are available, the condition for a state being in the goal region is that that distance is less than a specified threshold. If this is indeed the case, ompl::base::GoalRegion (which inherits from ompl::base::Goal) should be used. The additional function ompl::base::GoalRegion::distanceGoal() needs to be implemented. However, all the versions of ompl::base::Goal::isSatisfied() are implemented in terms of this new function and a specified threshold (which is equal to the machine epsilon, by default).

~~~{.cpp}
class MyGoalRegion : public ompl::base::GoalRegion
{
public:
    MyGoalRegion(const SpaceInformationPtr &si) : ompl::base::GoalRegion(si)
    {
        setThreshold(0.1);
    }

    virtual double distanceGoal(const State *st) const
        {
        // perform any operations and return a double indicating the distance to the goal
    }
};
~~~

- For bi-directional planners to work, they need to know states in the goal region. If a means to sample the goal region is available, ompl::base::GoalSampleableRegion should be used. This class, which inherits from ompl::base::GoalRegion, defines two additional functions: ompl::base::GoalSampleableRegion::sampleGoal() and ompl::base::GoalSampleableRegion::maxSampleCount(). These functions allow the planners to sample goals and to tell how many different samples can be obtained in the goal region at most, respectively. Keep in mind that ompl::base::GoalRegion::distanceGoal() still needs to be implemented.

  At this point, it is interesting to remark that bi-directional planners do not need to check whether a state is in the goal region, since they start with such states, so calls to ompl::base::Goal::isSatisfied() are not made. Instead, when the trees are connected, a call is made to ompl::base::Goal::isStartGoalPairValid(). This function, which by default always returns true, tells the planner whether a particular start state should be considered as forming a valid path if connected to a particular goal state. Typically, changing this function is not needed. There are a few implementations of ompl::base::GoalSampleableRegion available, for convenience:

  - ompl::base::GoalState (inherits from ompl::base::GoalSampleableRegion) stores one state as the goal. Sampling the goal state will always return this state and the distance to the goal is implemented by calling ompl::base::StateSpace::distance() between the stored goal state and the state passed to ompl::base::GoalRegion::distanceGoal().
  - ompl::base::GoalStates (inherits from ompl::base::GoalSampleableRegion) is a generalization of ompl::base::GoalState. An array of goal states is stored. Sampling goals will cycle through the stored states. The implementation ompl::base::GoalRegion::distanceGoal() will return the minimum distance between the state passed as argument and the stored states. This computation is performed in linear time (in terms of number of stored states), so it may become costly for a large number of states.
  - ompl::base::GoalLazySamples (inherits from ompl::base::GoalStates). In case sampling of states is a time consuming process, this version of a goal allows the sampling process to take place in a separate thread, while the planner is running. As new states are found, they are added to the array of stored states. This is useful for example when performing inverse kinematics for an arm. The inverse kinematics computations can be performed while the planner is running. As more states are found, they become available to the planner.

## Using the Goal Region

Planners cast the specified goal representation into the minimal representation they can use. For uni-directional planners, ompl::base::Goal is fine. For bi-directional planners however, this needs to be ompl::base::GoalSampleableRegion.

If the planner can use the goal specification, it will compute a motion plan. If successful, the found path is stored in the goal region. Flags indicating whether the solution was approximate are also set. The user can query all the information the planner has set using accessors from ompl::base::Goal.
