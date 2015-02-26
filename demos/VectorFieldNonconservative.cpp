/**
 * VectorFieldNonconservative.cpp
 * COMP 450 Project 5
 * 25 November 2014 
 * Caleb Voss (cav2) & Wilson Beebe (wsb1)
 */

#include <fstream>

#include <ompl/base/StateSpace.h>
#include <ompl/base/objectives/VFUpstreamCriterionOptimizationObjective.hpp>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/VFRRT.h>
#include <ompl/geometric/SimpleSetup.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

/** No obstacles. */
bool isStateValid (const ob::State *state)
{
    return true;
}

/** Basic unit-norm rotation field. */
Eigen::VectorXd getVector (ob::SpaceInformationPtr &si, const ob::State *state)
{
    ob::ScopedState<> sstate(si);
    sstate = state;
    Eigen::VectorXd v(2);
    v[0] = sstate[1];
    v[1] = -sstate[0];
    v.normalize();
    return v;
}

int main(int argc, char **argv)
{
    // construct the state space we are planning in
    ob::StateSpacePtr space(new ob::RealVectorStateSpace(2));
    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));

    ob::RealVectorBounds bounds(2);
    bounds.setLow(-10);
    bounds.setHigh(10);

    space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

    // define a simple setup class
    og::SimpleSetup ss(space);

    // set state validity checking for this space
    ss.setStateValidityChecker(&isStateValid);

    // create a start state
    ob::ScopedState<> start(space);
    start[0] = -5;
    start[1] = -2;

    // create a goal state
    ob::ScopedState<> goal(space);
    goal[0] = 5;
    goal[1] = 2;

    // set the start and goal states
    ss.setStartAndGoalStates(start, goal, 0.1);
    
    // make the vector field
    og::VFRRT::VectorField field(ss.getStateSpace(), boost::bind(&getVector, ss.getSpaceInformation(), _1));
    
    ob::OptimizationObjectivePtr opt(new ob::VFUpstreamCriterionOptimizationObjective(ss.getSpaceInformation(), &field));
    ss.getProblemDefinition()->setOptimizationObjective(opt);
    
    // initialize the planner
    double explorationSetting = 0.7;
    double lambda = 1;
    unsigned int update_freq = 100;
    ss.setPlanner(ob::PlannerPtr(new og::VFRRT(ss.getSpaceInformation(), field, explorationSetting, lambda, update_freq)));
    ss.setup();

    // attempt to solve the problem
    ob::PlannerStatus solved = ss.solve(10.0);

    if (solved)
    {
        if (solved == ob::PlannerStatus::EXACT_SOLUTION)
            std::cout << "Found solution.\n";
        else
            std::cout << "Found approximate solution.\n";
        
        // Set up to write the path
        std::ofstream f("vfrrt-nonconservative.path");
        
        ompl::geometric::PathGeometric p = ss.getSolutionPath();
        ob::ScopedState<> state(ss.getStateSpace());
        double cost = 0;
        ob::OptimizationObjectivePtr opt(new ob::VFUpstreamCriterionOptimizationObjective(ss.getSpaceInformation(), &field));
        for (std::size_t i = 0; i < p.getStateCount(); i++)
        {
            // Write to file
            state = p.getState(i);
            f << state[0] << " " << state[1] << "\n";
            
            if (i > 0)
                cost += opt->motionCost(p.getState(i-1), p.getState(i)).value();
        }
        std::cout << "Total cost: " << cost << "\n";
    }
    else
        std::cout << "No solution found.\n";

    return 0;
}
