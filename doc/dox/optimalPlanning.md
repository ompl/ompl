# Optimal Planning

In some motion planning problems, you might not want just _any_ valid path between your start and goal states. You might be interested in the shortest path, or perhaps the path that steers the farthest away from obstacles. In these cases you're looking for an _optimal_ path: a path which satisfies your constraints (connects start and goal states without collisions) and also optimizes some path quality metric. Path length and path clearance are two examples of path quality metrics. Motion planners which attempt to optimize path quality metrics are known as _optimal planners_.

In order to perform optimal planning, you need two things:

1. A path quality metric, or _optimization objective_.
2. An optimal motion planner

You can specify a path quality metric using the `ompl::base::OptimizationObjective` class. As for the optimal planner, OMPL currently provides three optimal motion planners: 

- PRMstar
- RRTstar
- TRRT

Defining an optimal motion planning problem is almost exactly the same as defining a regular motion planning problem, with two main differences:
1. You need to specify an `OptimizationObjective` to the `ProblemDefinition`.
2. You need to use an optimal planner for the actual motion planning.

## Optimization Objectives

OMPL comes with several predefined optimization objectives:

- path length
- minimum path clearance
- general state cost integral
- mechanical work

Each of these optimization objectives defines a notion of the _cost_ of a path. In OMPL we define cost as a value accrued over a motion in the robot's configuration space.  In geometric planning, a motion is fully defined by a start state and an end state. By default, these objectives attempt to minimize the path cost, but this behaviour can be customized. We also assume that the cost of an entire path can be broked down into an accumulation of the costs of the smaller motions that make up the path; the method of accumulation (e.g. summation, multiplication, etc.) can be customized.

OMPL also provides users with the ability to combine objectives for multiobjective optimal planning problems by using the `ompl::base::MultiOptimizationObjective` class.

## Limitations

OMPL assumes that the cost of a path can be represented with one `double` value. In many problems one value will suffice to fully define the optimization objective, even if we're working with a multiobjective problem. In these cases, the cost of a path can be represented as a weighted sum of the costs of the path under each of the individual objectives which make up the multiobjective.

However, there are problems that cannot be represented with this assumption. For instance, OMPL cannot represent a multiobjective which combines the following two objectives:

- Path length
- minimum clearance

The reason why these two objectives cannot be combined is because we need more than one value to perform the accumulation of the path cost. We need one value to hold the accumulation of length along the path, and another value to hold the the minimum clearance value encountered so far in the path. Therefore, the multiobjective problems that cannot be represented in OMPL are those where the individual objectives do not share a cost accumulation function.
