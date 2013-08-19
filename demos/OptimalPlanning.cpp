/* Author: Luis G. Torres */

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

// Our "collision checker". For this demo, our robot's state space
// lies in [0,1]x[0,1], with a circular obstacle of radius 0.25
// centered at (0.5,0.5). Any states lying in this circular region are
// considered "in collision".
bool isStateValid(const ob::State* state)
{
    // We know we're working with a RealVectorStateSpace in this
    // example, so we downcast state into the specific type.
    const ob::RealVectorStateSpace::StateType* state2D = 
        state->as<ob::RealVectorStateSpace::StateType>();

    // Extract the robot's (x,y) position from its state
    double x = state2D->values[0];
    double y = state2D->values[1];

    // Check whether (x,y) is within the circular obstacle of radius 0.25
    // centered at (0.5,0.5); equivalently, we check whether the squared
    // distance between (x,y) and (0.5,0.5) is greater than 0.25^2.
    return (x-0.5)*(x-0.5) + (y-0.5)*(y-0.5) >= 0.25*0.25;
}

// Returns a structure representing the optimization objective to use
// for optimal motion planning. This method returns an objective which
// attempts to minimize the length in configuration space of computed
// paths.
ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
    return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
}

ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
{
    ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
    obj->setCostThreshold(ob::Cost(1.51));
    return obj;
}

void plan(void)
{
    // Construct the robot state space in which we're planning. We're
    // planning in [0,1]x[0,1], a subset of R^2.
    ob::StateSpacePtr space(new ob::RealVectorStateSpace(2));

    // Set the bounds of space to be in [0,1].
    space->as<ob::RealVectorStateSpace>()->setBounds(0.0, 1.0);

    // Construct a space information instance for this state space
    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));

    // Set the function used to check which states in the space are valid
    si->setStateValidityChecker(boost::bind(&isStateValid, _1));

    // Set our robot's starting state to be the bottom-left corner of
    // the environment, or (0,0).
    ob::ScopedState<> start(space);
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = 0.0;
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = 0.0;

    // Set our robot's goal state to be the top-right corner of the
    // environment, or (1,1).
    ob::ScopedState<> goal(space);
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = 1.0;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = 1.0;

    // Create a problem instance
    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

    // Set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // Since we want to find an optimal plan, we need to define what
    // is optimal with an OptimizationObjective structure. This method
    // returns an objective where the optimal path is the one that is
    // shortest in length.
    //
    // NOTE: The optimal planner we're using, RRTstar, actually
    // defaults to optimizing path length if no objective is specified
    // to the ProblemDefinition, meaning this next statement is not
    // actually necessary.
    // pdef->setOptimizationObjective(getPathLengthObjective(si));
    pdef->setOptimizationObjective(getThresholdPathLengthObj(si));

    // Construct our optimal planner using the RRTstar algorithm.
    ob::PlannerPtr optimalPlanner(new og::RRTstar(si));

    // Set the problem instance for our planner to solve
    optimalPlanner->setProblemDefinition(pdef);
    optimalPlanner->setup();

    // attempt to solve the planning problem within one second of
    // planning time
    ob::PlannerStatus solved = optimalPlanner->solve(1.0);

    if (solved)
    {
        // Output the length of the path found
        std::cout 
            << "Found solution of path length " 
            << pdef->getSolutionPath()->length() << std::endl;
    }
    else
        std::cout << "No solution found." << std::endl;
}

int main()
{
    plan();

    return 0;
}
