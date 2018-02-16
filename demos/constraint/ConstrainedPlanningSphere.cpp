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

#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/est/BiEST.h>
#include <ompl/geometric/planners/est/ProjEST.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>

namespace po = boost::program_options;
namespace ob = ompl::base;
namespace og = ompl::geometric;

enum SPACE_TYPE
{
    PJ,
    AT,
    TB
};

enum PLANNER_TYPE
{
    RRT,
    RRTConnect,
    RRTstar,
    EST,
    BiEST,
    ProjEST,
    BITstar,
    PRM,
    SPARS,
    KPIECE,
    BKPIECE
};

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

class SphereProjection : public ob::ProjectionEvaluator
{
public:
    SphereProjection(ob::StateSpacePtr space) : ob::ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        return 2;
    }

    void defaultCellSizes() override
    {
        cellSizes_.resize(2);
        cellSizes_[0] = 0.1;
        cellSizes_[1] = 0.1;
    }

    void project(const ob::State *state, ob::EuclideanProjection &projection) const override
    {
        auto &&x = state->as<ompl::base::ConstrainedStateSpace::StateType>()->constVectorView();
        projection(0) = atan2(x[1], x[0]);
        projection(1) = acos(x[2]);
    }
};

bool obstacles(const ob::State *state)
{
    auto &&x = state->as<ob::ConstrainedStateSpace::StateType>()->constVectorView();

    if (-0.80 < x[2] && x[2] < -0.55)
    {
        if (-0.05 < x[1] && x[1] < 0.05)
            return x[0] > 0;
        return false;
    }
    else if (-0.15 < x[2] && x[2] < 0.15)
    {
        if (-0.05 < x[0] && x[0] < 0.05)
            return x[1] < 0;
        return false;
    }
    else if (0.55 < x[2] && x[2] < 0.80)
    {
        if (-0.05 < x[1] && x[1] < 0.05)
            return x[0] < 0;
        return false;
    }

    return true;
}

void spherePlanning(bool output, enum SPACE_TYPE space, enum PLANNER_TYPE planner)
{
    // Create the ambient space state space for the problem.
    auto rvss = std::make_shared<ob::RealVectorStateSpace>(3);

    ob::RealVectorBounds bounds(3);
    bounds.setLow(-2);
    bounds.setHigh(2);

    rvss->setBounds(bounds);

    // Create a shared pointer to our constraint.
    auto constraint = std::make_shared<SphereConstraint>();

    ob::ConstrainedStateSpacePtr css;
    ob::ConstrainedSpaceInformationPtr csi;

    // Combine the ambient state space and the constraint to create the
    // constrained state space.
    switch (space)
    {
        case PJ:
            OMPL_INFORM("Using Projection-Based State Space!");
            css = std::make_shared<ob::ProjectedStateSpace>(rvss, constraint);
            csi = std::make_shared<ob::ConstrainedSpaceInformation>(css);
            break;
        case AT:
            OMPL_INFORM("Using Atlas-Based State Space!");
            css = std::make_shared<ob::AtlasStateSpace>(rvss, constraint);
            csi = std::make_shared<ob::ConstrainedSpaceInformation>(css);
            break;
        case TB:
            OMPL_INFORM("Using Tangent Bundle-Based State Space!");
            css = std::make_shared<ob::TangentBundleStateSpace>(rvss, constraint);
            csi = std::make_shared<ob::TangentBundleSpaceInformation>(css);
            break;
    }

    // Setup space information and setup
    auto ss = std::make_shared<og::SimpleSetup>(csi);

    // Create start and goal states (poles of the sphere)
    ob::ScopedState<> start(css);
    ob::ScopedState<> goal(css);

    switch (space)
    {
        case PJ:
            start->as<ob::ProjectedStateSpace::StateType>()->vectorView() << 0, 0, -1;
            goal->as<ob::ProjectedStateSpace::StateType>()->vectorView() << 0, 0, 1;
            break;
        case AT:
        case TB:
            start->as<ob::AtlasStateSpace::StateType>()->vectorView() << 0, 0, -1;
            goal->as<ob::AtlasStateSpace::StateType>()->vectorView() << 0, 0, 1;
            css->as<ob::AtlasStateSpace>()->anchorChart(start.get());
            css->as<ob::AtlasStateSpace>()->anchorChart(goal.get());
            break;
    }

    css->registerProjection("sphere", std::make_shared<SphereProjection>(css));

    // Create planner

    ob::PlannerPtr pp;
    switch (planner)
    {
        case RRT:
            pp = std::make_shared<og::RRT>(csi);
            break;
        case RRTConnect:
            pp = std::make_shared<og::RRTConnect>(csi);
            break;
        case RRTstar:
            pp = std::make_shared<og::RRTstar>(csi);
            break;
        case EST:
            pp = std::make_shared<og::EST>(csi);
            break;
        case BiEST:
            pp = std::make_shared<og::BiEST>(csi);
            break;
        case ProjEST:
        {
            auto est = std::make_shared<og::ProjEST>(csi);
            est->setProjectionEvaluator("sphere");
            pp = est;
            break;
        }
        case BITstar:
            pp = std::make_shared<og::BITstar>(csi);
            break;
        case PRM:
            pp = std::make_shared<og::PRM>(csi);
            break;
        case SPARS:
            pp = std::make_shared<og::SPARS>(csi);
            break;
        case KPIECE:
        {
            auto kpiece = std::make_shared<og::KPIECE1>(csi);
            kpiece->setProjectionEvaluator("sphere");
            pp = kpiece;
            break;
        }
        case BKPIECE:
        {
            auto kpiece = std::make_shared<og::BKPIECE1>(csi);
            kpiece->setProjectionEvaluator("sphere");
            pp = kpiece;
            break;
        }
    }

    // Setup problem
    ss->setStartAndGoalStates(start, goal);
    ss->setStateValidityChecker(obstacles);
    ss->setPlanner(pp);
    ss->setup();

    // Solve the problem
    ob::PlannerStatus stat = ss->solve(5.);
    std::cout << std::endl;

    if (stat)
    {
        // Get solution and validate
        auto path = ss->getSolutionPath();
        if (!path.check())
            OMPL_WARN("Path does not satisfy constraints!");

        if (stat == ob::PlannerStatus::APPROXIMATE_SOLUTION)
            OMPL_WARN("Solution is approximate.");

        // Simplify solution and validate simplified solution path.
        OMPL_INFORM("Simplifying solution...");
        ss->simplifySolution(5.);

        auto simplePath = ss->getSolutionPath();
        OMPL_INFORM("Simplified Path Length: %.3f -> %.3f", path.length(), simplePath.length());

        if (!simplePath.check())
            OMPL_WARN("Simplified path does not satisfy constraints!");

        // For atlas types, output information about size of atlas and amount of space explored
        if (space == AT || space == TB)
        {
            auto at = css->as<ompl::base::AtlasStateSpace>();
            OMPL_INFORM("Atlas has %zu charts", at->getChartCount());
            if (space == AT)
                OMPL_INFORM("Atlas has %.3f%% openness", at->estimateFrontierPercent());
        }

        if (output)
        {
            // Interpolate and validate interpolated solution path.
            OMPL_INFORM("Interpolating path...");
            simplePath.interpolate();

            if (!simplePath.check())
                OMPL_WARN("Interpolated path does not satisfy constraints!");

            OMPL_INFORM("Dumping path to `sphere_path.txt`.");
            std::ofstream pathfile("sphere_path.txt");
            simplePath.printAsMatrix(pathfile);
            pathfile.close();
        }
    }
    else
        OMPL_WARN("No solution found.");

    if (output)
    {
        OMPL_INFORM("Dumping planner graph to `sphere_graph.graphml`.");
        ob::PlannerData data(csi);
        pp->getPlannerData(data);

        std::ofstream graphfile("sphere_graph.graphml");
        data.printGraphML(graphfile);
        graphfile.close();

        if (space == AT || space == TB)
        {
            OMPL_INFORM("Dumping atlas to `sphere_atlas.ply`.");
            std::ofstream atlasfile("sphere_atlas.ply");
            css->as<ob::AtlasStateSpace>()->printPLY(atlasfile);
            atlasfile.close();
        }
    }
}

auto help_msg = "Shows this help message.";
auto output_msg = "Dump found solution path (if one exists) in plain text and planning graph in GraphML to "
                  "`sphere_path.txt` and `sphere_graph.graphml` respectively.";
auto space_msg = "Choose which constraint handling methodology to use. One of:\n"
                 "PJ - Projection (Default), "
                 "AT - Atlas, "
                 "TB - Tangent Bundle.";
auto planner_msg = "Choose which motion planner to use. One of:\n"
                   "RRT (Default), RRTConnect, RRTstar, EST, BITstar, PRM, KPIECE";

std::istream &operator>>(std::istream &in, enum SPACE_TYPE &type)
{
    std::string token;
    in >> token;
    if (token == "PJ")
        type = PJ;
    else if (token == "AT")
        type = AT;
    else if (token == "TB")
        type = TB;
    else
        in.setstate(std::ios_base::failbit);

    return in;
}

std::istream &operator>>(std::istream &in, enum PLANNER_TYPE &type)
{
    std::string token;
    in >> token;
    if (token == "RRT")
        type = RRT;
    else if (token == "RRTConnect")
        type = RRTConnect;
    else if (token == "RRTstar")
        type = RRTstar;
    else if (token == "EST")
        type = EST;
    else if (token == "BiEST")
        type = BiEST;
    else if (token == "ProjEST")
        type = ProjEST;
    else if (token == "BITstar")
        type = BITstar;
    else if (token == "PRM")
        type = PRM;
    else if (token == "SPARS")
        type = SPARS;
    else if (token == "KPIECE")
        type = KPIECE;
    else if (token == "BKPIECE")
        type = BKPIECE;
    else
        in.setstate(std::ios_base::failbit);

    return in;
}

int main(int argc, char **argv)
{
    bool output;
    enum SPACE_TYPE space = PJ;
    enum PLANNER_TYPE planner = RRT;

    po::options_description desc("Options");
    desc.add_options()("help,h", help_msg);
    desc.add_options()("output,o", po::bool_switch(&output)->default_value(false), output_msg);
    desc.add_options()("space,s", po::value<enum SPACE_TYPE>(&space), space_msg);
    desc.add_options()("planner,p", po::value<enum PLANNER_TYPE>(&planner), planner_msg);

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return 1;
    }

    spherePlanning(output, space, planner);

    return 0;
}
