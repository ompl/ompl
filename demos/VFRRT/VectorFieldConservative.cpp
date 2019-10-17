/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Caleb Voss and Wilson Beebe
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Authors: Caleb Voss, Wilson Beebe */

#include <fstream>

#include <ompl/base/StateSpace.h>
#include <ompl/base/objectives/VFMechanicalWorkOptimizationObjective.h>
#include <ompl/base/objectives/VFUpstreamCriterionOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/rrt/VFRRT.h>
#include <ompl/geometric/SimpleSetup.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

enum PlannerType { VFRRT = 0, TRRT, RRTSTAR };

/** Gradient of the potential function 1 + sin(x[0]) * sin(x[1]). */
Eigen::VectorXd field(const ob::State *state)
{
    const ob::RealVectorStateSpace::StateType &x = *state->as<ob::RealVectorStateSpace::StateType>();
    Eigen::VectorXd v(2);
    v[0] = std::cos(x[0]) * std::sin(x[1]);
    v[1] = std::sin(x[0]) * std::cos(x[1]);
    return -v;
}

/** Make a problem using a conservative vector field. */
og::SimpleSetupPtr setupProblem(PlannerType plannerType)
{
    // construct the state space we are planning in
    auto space(std::make_shared<ob::RealVectorStateSpace>(2));
    auto si(std::make_shared<ob::SpaceInformation>(space));

    ob::RealVectorBounds bounds(2);
    bounds.setLow(-10);
    bounds.setHigh(10);

    space->setBounds(bounds);

    // define a simple setup class
    auto ss(std::make_shared<og::SimpleSetup>(space));

    // set state validity checking for this space
    ss->setStateValidityChecker(std::make_shared<ob::AllValidStateValidityChecker>(si));

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

    // make the optimization objectives for TRRT and RRT*, and set the planner
    if (plannerType == TRRT)
    {
        ss->setOptimizationObjective(
            std::make_shared<ob::VFMechanicalWorkOptimizationObjective>(si, field));
        ss->setPlanner(std::make_shared<og::TRRT>(ss->getSpaceInformation()));
    }
    else if (plannerType == RRTSTAR)
    {
        ss->setOptimizationObjective(
            std::make_shared<ob::VFUpstreamCriterionOptimizationObjective>(si, field));
        ss->setPlanner(std::make_shared<og::RRTstar>(ss->getSpaceInformation()));
    }
    else if (plannerType == VFRRT)
    {
        double explorationSetting = 0.7;
        double lambda = 1;
        unsigned int update_freq = 100;
        ss->setPlanner(std::make_shared<og::VFRRT>(ss->getSpaceInformation(), field, explorationSetting, lambda, update_freq));
    }
    else
    {
        std::cout << "Bad problem number.\n";
        exit(-1);
    }

    ss->setup();

    return ss;
}

/** Get the filename to write the path to. */
std::string problemName(PlannerType plannerType)
{
    if (plannerType == VFRRT)
        return std::string("vfrrt-conservative.path");
    if (plannerType == TRRT)
        return std::string("trrt-conservative.path");
    else if (plannerType == RRTSTAR)
        return std::string("rrtstar-conservative.path");
    else
    {
        std::cout << "Bad problem number.\n";
        exit(-1);
    }
}

int main(int, char **)
{
    // Run all three problems
    for (unsigned int n = 0; n < 3; n++)
    {
        // initialize the planner
        og::SimpleSetupPtr ss = setupProblem(PlannerType(n));

        // attempt to solve the problem
        ob::PlannerStatus solved = ss->solve(10.0);

        if (solved)
        {
            if (solved == ob::PlannerStatus::EXACT_SOLUTION)
                std::cout << "Found solution.\n";
            else
                std::cout << "Found approximate solution.\n";

            // Set up to write the path
            std::ofstream f(problemName(PlannerType(n)).c_str());
            ompl::geometric::PathGeometric p = ss->getSolutionPath();
            p.interpolate();
            auto upstream(std::make_shared<ob::VFUpstreamCriterionOptimizationObjective>(
                ss->getSpaceInformation(), field));
            p.printAsMatrix(f);
            std::cout << "Total upstream cost: " << p.cost(upstream) << "\n";
        }
        else
            std::cout << "No solution found.\n";
    }

    return 0;
}
