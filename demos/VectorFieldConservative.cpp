/**
 * VectorFieldConservative.cpp
 * COMP 450 Project 5
 * 25 November 2014 
 * Caleb Voss (cav2) & Wilson Beebe (wsb1)
 */

#include <fstream>

#include <ompl/base/StateSpace.h>
#include <ompl/base/objectives/VFMechanicalWorkOptimizationObjective.hpp>
#include <ompl/base/objectives/VFUpstreamCriterionOptimizationObjective.hpp>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/rrt/VFRRT.h>
#include <ompl/geometric/SimpleSetup.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

/** No obstacles. */
bool isStateValid (const ob::State *state)
{
    return true;
}

/** Potential function over the space. */
double getPotential (const Eigen::VectorXd &x)
{
    return 1 + std::sin(x[0]) * std::sin(x[1]);
}

/** Gradient of the potential function. */
Eigen::VectorXd getGradient (const Eigen::VectorXd &x)
{
    Eigen::VectorXd g(2);
    g[0] = std::cos(x[0]) * std::sin(x[1]);
    g[1] = std::sin(x[0]) * std::cos(x[1]);
    return g;
}

/** Get the vector of the field associated to a particular state. */
Eigen::VectorXd getVector (ob::SpaceInformationPtr &si, const ob::State *state)
{
    ob::ScopedState<> sstate(si);
    sstate = state;
    int d = sstate.reals().size();
    Eigen::VectorXd v(d);
    for (int i = 0; i < d; i++)
    {
        v[i] = sstate[i];
    }
    
    return -getGradient(v);
}

#define _VF_RRT 0
#define _TRRT 1
#define _RRTstar 2

/** Make a problem using a conservative vector field. */
og::SimpleSetupPtr setupProblem(int probNum)
{
    // construct the state space we are planning in
    ob::StateSpacePtr space(new ob::RealVectorStateSpace(2));
    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));

    ob::RealVectorBounds bounds(2);
    bounds.setLow(-10);
    bounds.setHigh(10);

    space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

    // define a simple setup class
    og::SimpleSetupPtr ss(new og::SimpleSetup(space));

    // set state validity checking for this space
    ss->setStateValidityChecker(&isStateValid);

    // create a start state
    ob::ScopedState<> start(space);
    start[0] = -5;
    start[1] = -2;

    // create a goal state
    ob::ScopedState<> goal(space);
    goal[0] = 5;
    goal[1] = 3;

    // set the start and goal states
    ss->setStartAndGoalStates(start, goal, 0.1);
    
    // make the vector field
    og::VFRRT::VectorField *field = new og::VFRRT::VectorField(ss->getStateSpace(), boost::bind(&getVector, ss->getSpaceInformation(), _1));
    
    // make the optimization objectives for TRRT and RRT*, and set the planner
    if (probNum == _TRRT)
    {
        ob::OptimizationObjectivePtr opt(new ob::VFMechanicalWorkOptimizationObjective(si, field));
        ss->getProblemDefinition()->setOptimizationObjective(opt);
        ss->setPlanner(ob::PlannerPtr(new og::TRRT(ss->getSpaceInformation())));
    }
    else if (probNum == _RRTstar)
    {
        ob::OptimizationObjectivePtr opt(new ob::VFUpstreamCriterionOptimizationObjective(si, field));
        ss->getProblemDefinition()->setOptimizationObjective(opt);
        ss->setPlanner(ob::PlannerPtr(new og::RRTstar(ss->getSpaceInformation())));
    }
    else if (probNum == _VF_RRT)
    {
        double explorationSetting = 0.7;
        double lambda = 1;
        unsigned int update_freq = 100;
        ss->setPlanner(ob::PlannerPtr(new og::VFRRT(ss->getSpaceInformation(), *field, explorationSetting, lambda, update_freq)));
    }
    else
    {
        std::cout << "Bad problem number.\n";
        exit(0);
    }
    
    ss->setup();
    
    return ss;
}

/** Get the filename to write the path to. */
std::string problemName(int probNum)
{
    if (probNum == _VF_RRT)
        return std::string("vfrrt-conservative.path");
    else if (probNum == _TRRT)
        return std::string("trrt-conservative.path");
    else if (probNum == _RRTstar)
        return std::string("rrtstar-conservative.path");
    else
    {
        std::cout << "Bad problem number.\n";
        exit(0);
    }
}

int main(int argc, char **argv)
{
    // Run all three problems
    for (int n = _VF_RRT; n <= _RRTstar; n++)
    {
        // initialize the planner
        og::SimpleSetupPtr ss = setupProblem(n);

        // attempt to solve the problem
        ob::PlannerStatus solved = ss->solve(10.0);

        if (solved)
        {
            if (solved == ob::PlannerStatus::EXACT_SOLUTION)
                std::cout << "Found solution.\n";
            else
                std::cout << "Found approximate solution.\n";
            
            // Set up to write the path
            std::ofstream f(problemName(n).c_str());
            
            ompl::geometric::PathGeometric p = ss->getSolutionPath();
            p.interpolate();
            ob::ScopedState<> state(ss->getStateSpace());
            double cost = 0;
            og::VFRRT::VectorField field(ss->getStateSpace(), boost::bind(&getVector, ss->getSpaceInformation(), _1));
            ob::OptimizationObjectivePtr upstream(new ob::VFUpstreamCriterionOptimizationObjective(ss->getSpaceInformation(), &field));
            for (std::size_t i = 0; i < p.getStateCount(); i++)
            {
                // Write to file
                state = p.getState(i);
                f << state[0] << " " << state[1] << "\n";
                
                if (i > 0)
                    cost += upstream->motionCost(p.getState(i-1), p.getState(i)).value();
            }
            std::cout << "Total cost: " << cost << "\n";
        }
        else
            std::cout << "No solution found.\n";
    }

    return 0;
}
