/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, Rice University
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
*   * Neither the name of the Rice University nor the names of its
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

/* Author: Zachary Kingston */

#include <iostream>
#include <fstream>

#include <boost/program_options.hpp>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>

#include <ompl/base/Constraint.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>
#include <ompl/base/spaces/constraint/AtlasChart.h>
#include <ompl/base/spaces/constraint/AtlasStateSpace.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>

#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>

namespace po = boost::program_options;
namespace ob = ompl::base;
namespace og = ompl::geometric;

class SphereConstraint : public ob::Constraint
{
public:
    SphereConstraint() : ob::Constraint(3, 1)
    {
    }

    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
    {
        out[0] = x.norm() - 1;
    }

    void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override
    {
        out = x.transpose().normalized();
    }
};

bool isValid(const ob::State *state)
{
    auto x = state->as<ob::ConstrainedStateSpace::StateType>()->constVectorView();

    if (-0.75 < x[2] && x[2] < -0.60)
    {
        if (-0.05 < x[1] && x[1] < 0.05)
            return x[0] > 0;
        return false;
    }
    else if (-0.1 < x[2] && x[2] < 0.1)
    {
        if (-0.05 < x[0] && x[0] < 0.05)
            return x[1] < 0;
        return false;
    }
    else if (0.60 < x[2] && x[2] < 0.75)
    {
        if (-0.05 < x[1] && x[1] < 0.05)
            return x[0] < 0;
        return false;
    }

    return true;
}

void spherePlanning(bool output)
{
    // Create the ambient space state space for the problem.
    auto rvss = std::make_shared<ob::RealVectorStateSpace>(3);

    ob::RealVectorBounds bounds(3);
    bounds.setLow(-1);
    bounds.setHigh(1);

    rvss->setBounds(bounds);

    // Create a shared pointer to our constraint.
    auto constraint = std::make_shared<SphereConstraint>();

    // Combine the ambient state space and the constraint to create the
    // constrained state space.
    auto css = std::make_shared<ob::ProjectedStateSpace>(rvss, constraint);

    // Setup space information and setup
    auto csi = std::make_shared<ob::ConstrainedSpaceInformation>(css);
    auto ss = std::make_shared<og::SimpleSetup>(csi);

    // Create start and goal states (poles of the sphere)
    ob::ScopedState<> start(css);
    ob::ScopedState<> goal(css);
    start->as<ob::ProjectedStateSpace::StateType>()->vectorView() << 0, 0, -1;
    goal->as<ob::ProjectedStateSpace::StateType>()->vectorView() << 0, 0, 1;

    // Create planner
    auto planner = std::make_shared<og::RRTConnect>(csi);
    planner->setRange(0.1);

    // Setup problem
    ss->setStartAndGoalStates(start, goal);
    ss->setStateValidityChecker(isValid);
    ss->setPlanner(planner);
    ss->setup();

    ob::PlannerStatus stat = ss->solve(5.);
    if (stat)
    {
        // Get solution and validate
        auto path = ss->getSolutionPath();
        if (!css->checkPath(path))
            std::cout << "Path does not satisfy constraints!" << std::endl;

        if (stat == ob::PlannerStatus::APPROXIMATE_SOLUTION)
            std::cout << "Solution is approximate." << std::endl;

        // Simplify solution and validate simplified solution path.
        std::cout << "Simplifying solution..." << std::endl;
        ss->simplifySolution(5.);

        auto simplePath = ss->getSolutionPath();
        std::cout << "Path Length " << path.length() << " -> " << simplePath.length() << std::endl;

        if (!css->checkPath(simplePath))
            std::cout << "Simplified path does not satisfy constraints!" << std::endl;

        if (output)
        {
            // Interpolate and validate interpolated solution path.
            std::cout << "Interpolating path..." << std::endl;
            simplePath.interpolate();

            if (!css->checkPath(simplePath))
                std::cout << "Interpolated path does not satisfy constraints!" << std::endl;

            std::cout << "Dumping path..." << std::endl;
            std::ofstream pathfile("sphere_path.txt");
            simplePath.printAsMatrix(pathfile);
            pathfile.close();
        }
    }
    else
        std::cout << "No solution found." << std::endl;

    if (output)
    {
        std::cout << "Dumping graph data..." << std::endl;
        ob::PlannerData data(csi);
        planner->getPlannerData(data);

        std::ofstream graphfile("sphere_graph.graphml");
        data.printGraphML(graphfile);
        graphfile.close();
    }
}

auto help_msg = "Shows this help message.";
auto output_msg = "Dump found solution path (if one exists) in plain text and planning graph in GraphML to "
                  "`sphere_path.txt` and `sphere_graph.graphml` respectively.";

int main(int argc, char **argv)
{
    bool output;

    po::options_description desc("Options");
    desc.add_options()("help,h", help_msg)("output,o", po::bool_switch(&output)->default_value(false), output_msg);

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return 1;
    }

    spherePlanning(output);

    return 0;
}
