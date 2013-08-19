/* Author: Luis G. Torres */

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <fstream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

// Our "collision checker". For this demo, our robot's state space
// lies in [0,1]x[0,1], with a circular obstacle of radius 0.25
// centered at (0.5,0.5). Any states lying in this circular region are
// considered "in collision".
class ValidityChecker : public ob::StateValidityChecker
{
public:
    ValidityChecker(const ob::SpaceInformationPtr& si) :
        ob::StateValidityChecker(si) {}

    // Returns whether the given state's position overlaps the
    // circular obstacle
    bool isValid(const ob::State* state) const
    {
        return this->clearance(state) > 0.0;
    }

    // Returns the distance from the given state's position to the
    // boundary of the circular obstacle.
    double clearance(const ob::State* state) const
    {
        // We know we're working with a RealVectorStateSpace in this
        // example, so we downcast state into the specific type.
        const ob::RealVectorStateSpace::StateType* state2D = 
            state->as<ob::RealVectorStateSpace::StateType>();

        // Extract the robot's (x,y) position from its state
        double x = state2D->values[0];
        double y = state2D->values[1];

        // Distance formula between two points, offset by the circle's
        // radius
        return sqrt((x-0.5)*(x-0.5) + (y-0.5)*(y-0.5)) - 0.25;
    }
};

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

ob::OptimizationObjectivePtr getClearanceObjective(const ob::SpaceInformationPtr& si);

ob::OptimizationObjectivePtr getBalancedObjective1(const ob::SpaceInformationPtr& si);

ob::OptimizationObjectivePtr getBalancedObjective2(const ob::SpaceInformationPtr& si);

void plan(int argc, char** argv)
{
    // Construct the robot state space in which we're planning. We're
    // planning in [0,1]x[0,1], a subset of R^2.
    ob::StateSpacePtr space(new ob::RealVectorStateSpace(2));

    // Set the bounds of space to be in [0,1].
    space->as<ob::RealVectorStateSpace>()->setBounds(0.0, 1.0);

    // Construct a space information instance for this state space
    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));

    // Set the object used to check which states in the space are valid
    si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si)));

    si->setup();

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
    // pdef->setOptimizationObjective(getThresholdPathLengthObj(si));
    // pdef->setOptimizationObjective(getClearanceObjective(si));
    // pdef->setOptimizationObjective(getBalancedObjective1(si));
    pdef->setOptimizationObjective(getBalancedObjective2(si));

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

        // If a filename was specified, output the path as a matrix to
        // that file for visualization
        if (argc > 1)
        {
            std::ofstream outFile(argv[1]);
            boost::dynamic_pointer_cast<og::PathGeometric>(pdef->getSolutionPath())->
                printAsMatrix(outFile);
            outFile.close();
        }
    }
    else
        std::cout << "No solution found." << std::endl;
}

int main(int argc, char** argv)
{
    plan(argc, argv);

    return 0;
}

/** Defines an optimization objective which attempts to steer the
    robot away from obstacles. To formulate this objective as a
    minimization of path cost, we can define the cost of a path as a
    summation of the costs of each of the states along the path, where
    each state cost is a function of that state's clearance from
    obstacles.

    The class StateCostIntegralObjective represents objectives as
    summations of state costs, just like we require. All we need to do
    then is inherit from that base class and define our specific state
    cost function by overriding the stateCost() method.
 */
class ClearanceObjective : public ob::StateCostIntegralObjective
{
public:
    ClearanceObjective(const ob::SpaceInformationPtr& si) :
        ob::StateCostIntegralObjective(si)
    {
        this->enableMotionCostInterpolation();
    }

    // Our requirement is to maximize path clearance from obstacles,
    // but we want to represent the objective as a path cost
    // minimization. Therefore, we set each state's cost to be the
    // reciprocal of its clearance, so that as state clearance
    // increases, the state cost decreases.    
    ob::Cost stateCost(const ob::State* s) const
    {
        return ob::Cost(1 / si_->getStateValidityChecker()->clearance(s));
    }
};

ob::OptimizationObjectivePtr getClearanceObjective(const ob::SpaceInformationPtr& si)
{
    return ob::OptimizationObjectivePtr(new ClearanceObjective(si));
}

/** Create an optimization objective which attempts to optimize both
    path length and clearance. We do this by defining our individual
    objectives, then adding them to a MultiOptimizationObjective
    object. This results in an optimization objective where path cost
    is equivalent to adding up each of the individual objectives' path
    costs. 

    When adding objectives, we can also optionally specify each
    objective's weighting factor to signify how important it is in
    optimal planning. If no weight is specified, the weight defaults to
    1.0.
*/
ob::OptimizationObjectivePtr getBalancedObjective1(const ob::SpaceInformationPtr& si)
{
    ob::OptimizationObjectivePtr lengthObj(new ob::PathLengthOptimizationObjective(si));
    ob::OptimizationObjectivePtr clearObj(new ClearanceObjective(si));

    ob::MultiOptimizationObjective* opt = new ob::MultiOptimizationObjective(si);
    opt->addObjective(lengthObj, 5.0);
    opt->addObjective(clearObj);

    return ob::OptimizationObjectivePtr(opt);
}

ob::OptimizationObjectivePtr getBalancedObjective2(const ob::SpaceInformationPtr& si)
{
    ob::OptimizationObjectivePtr lengthObj(new ob::PathLengthOptimizationObjective(si));
    ob::OptimizationObjectivePtr clearObj(new ClearanceObjective(si));

    return 5.0*lengthObj + clearObj;
}
