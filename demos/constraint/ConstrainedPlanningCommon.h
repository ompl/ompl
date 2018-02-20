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
#include <ompl/base/spaces/constraint/AtlasStateSpace.h>
#include <ompl/base/spaces/constraint/TangentBundleStateSpace.h>
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

auto space_msg = "Choose which constraint handling methodology to use. One of:\n"
                 "PJ - Projection (Default), "
                 "AT - Atlas, "
                 "TB - Tangent Bundle.";

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

void addSpaceOption(po::options_description &desc, enum SPACE_TYPE *space)
{
    desc.add_options()("space,s", po::value<enum SPACE_TYPE>(space), space_msg);
}

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

auto planner_msg = "Choose which motion planner to use. One of:\n"
                   "RRT (Default), RRTConnect, RRTstar, "
                   "EST, BiEST, ProjEST, "
                   "BITstar, "
                   "PRM, SPARS, "
                   "KPIECE, BKPIECE.";

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

void addPlannerOption(po::options_description &desc, enum PLANNER_TYPE *planner)
{
    desc.add_options()("planner,p", po::value<enum PLANNER_TYPE>(planner), planner_msg);
}

class ConstrainedProblem
{
public:
    ConstrainedProblem(enum SPACE_TYPE type, StateSpacePtr space, ConstraintPtr constraint)
      : space(std::move(space)), constraint(std::move(constraint))
    {
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

        ss = std::make_shared<og::SimpleSetup>(csi);
    }

    void setStartAndGoalStates(const Eigen::Ref<const Eigen::VectorXd> &start,
                               const Eigen::Ref<const Eigen::VectorXd> &goal)
    {
        // Create start and goal states (poles of the sphere)
        ob::ScopedState<> sstart(css);
        ob::ScopedState<> sgoal(css);

        sstart->as<ob::ProjectedStateSpace::StateType>()->vectorView() = start;
        sgoal->as<ob::ProjectedStateSpace::StateType>()->vectorView() = goal;

        switch (space)
        {
            case AT:
            case TB:
                css->as<ob::AtlasStateSpace>()->anchorChart(sstart.get());
                css->as<ob::AtlasStateSpace>()->anchorChart(sgoal.get());
                break;
            default:
                break;
        }

        // Setup problem
        ss->setStartAndGoalStates(sstart, sgoal);
    }

    void setPlanner(enum PLANNER_TYPE type, const std::string &projection = "")
    {
        const bool isProj = projection != "";

        switch (type)
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
                if (isProj)
                    est->setProjectionEvaluator(projection);
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
                if (isProj)
                    kpiece->setProjectionEvaluator(projection);
                pp = kpiece;
                break;
            }
            case BKPIECE:
            {
                auto kpiece = std::make_shared<og::BKPIECE1>(csi);
                if (isProj)
                    kpiece->setProjectionEvaluator(projection);
                pp = kpiece;
                break;
            }
        }

        ss->setPlanner(pp);
    }

    ob::StateSpacePtr space;
    ob::ConstraintPtr constraint;

    ob::ConstrainedStateSpacePtr css;
    ob::ConstrainedSpaceInformationPtr csi;

    ob::PlannerPtr pp;

    og::SimpleSetupPtr ss;
};
