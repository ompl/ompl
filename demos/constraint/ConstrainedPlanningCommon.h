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

#ifndef OMPL_DEMO_CONSTRAINED_COMMON_
#define OMPL_DEMO_CONSTRAINED_COMMON_

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
namespace om = ompl::magic;

enum SPACE_TYPE
{
    PJ = 0,
    AT = 1,
    TB = 2
};

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
    auto space_msg = "Choose which constraint handling methodology to use. One of:\n"
                     "PJ - Projection (Default), "
                     "AT - Atlas, "
                     "TB - Tangent Bundle.";

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
    auto planner_msg = "Choose which motion planner to use. One of:\n"
                       "RRT (Default), RRTConnect, RRTstar, "
                       "EST, BiEST, ProjEST, "
                       "BITstar, "
                       "PRM, SPARS, "
                       "KPIECE, BKPIECE.";

    desc.add_options()("planner,p", po::value<enum PLANNER_TYPE>(planner), planner_msg);
}

struct ConstrainedOptions
{
    double delta;
    double tolerance;
    unsigned int tries;
};

void addConstrainedOptions(po::options_description &desc, struct ConstrainedOptions *options)
{
    auto delta_msg = "Step-size for discrete geodesic on manifold.";
    auto tolerance_msg = "Constraint satisfaction tolerance.";
    auto tries_msg = "Maximum number sample tries per sample.";

    desc.add_options()("delta,d", po::value<double>(&options->delta)->default_value(om::CONSTRAINED_STATE_SPACE_DELTA),
                       delta_msg);
    desc.add_options()("tolerance",
                       po::value<double>(&options->tolerance)->default_value(om::CONSTRAINT_PROJECTION_TOLERANCE),
                       tolerance_msg);
    desc.add_options()(
        "tries", po::value<unsigned int>(&options->tries)->default_value(om::CONSTRAINT_PROJECTION_MAX_ITERATIONS),
        tries_msg);
}

struct AtlasOptions
{
    double epsilon;
    double rho;
    double exploration;
    double lambda;
    double alpha;
    bool bias;
    bool separate;
    unsigned int charts;
};

void addAtlasOptions(po::options_description &desc, struct AtlasOptions *options)
{
    auto epsilon_msg = "Maximum distance from an atlas chart to the manifold. Must be positive.";
    auto rho_msg = "Maximum radius for an atlas chart. Must be positive.";
    auto exploration_msg = "Value in [0, 1] which tunes balance of refinement and exploration in "
                           "atlas sampling.";
    auto lambda_msg = "Maximum `wandering` allowed during atlas traversal. Must be greater than 1.";
    auto alpha_msg = "Maximum angle between an atlas chart and the manifold. Must be in [0, PI/2].";
    auto bias_msg = "Sets whether the atlas should use frontier-biased chart sampling rather than "
                    "uniform.";
    auto separate_msg = "Sets that the atlas should not compute chart separating halfspaces.";
    auto charts_msg = "Maximum number of atlas charts that can be generated during one manifold "
                      "traversal.";

    desc.add_options()("epsilon", po::value<double>(&options->epsilon)->default_value(om::ATLAS_STATE_SPACE_EPSILON),
                       epsilon_msg);
    desc.add_options()("rho",
                       po::value<double>(&options->rho)
                           ->default_value(om::CONSTRAINED_STATE_SPACE_DELTA * om::ATLAS_STATE_SPACE_RHO_MULTIPLIER),
                       rho_msg);
    desc.add_options()("exploration",
                       po::value<double>(&options->exploration)->default_value(om::ATLAS_STATE_SPACE_EXPLORATION),
                       exploration_msg);
    desc.add_options()("lambda", po::value<double>(&options->lambda)->default_value(om::ATLAS_STATE_SPACE_LAMBDA),
                       lambda_msg);
    desc.add_options()("alpha", po::value<double>(&options->alpha)->default_value(om::ATLAS_STATE_SPACE_ALPHA),
                       alpha_msg);
    desc.add_options()("bias", po::bool_switch(&options->bias)->default_value(false), bias_msg);
    desc.add_options()("no-separate", po::bool_switch(&options->separate)->default_value(false), separate_msg);
    desc.add_options()(
        "charts",
        po::value<unsigned int>(&options->charts)->default_value(om::ATLAS_STATE_SPACE_MAX_CHARTS_PER_EXTENSION),
        charts_msg);
}

class ConstrainedProblem
{
public:
    ConstrainedProblem(enum SPACE_TYPE type, ob::StateSpacePtr space_, ob::ConstraintPtr constraint_)
      : type(type), space(std::move(space_)), constraint(std::move(constraint_))
    {
        // Combine the ambient state space and the constraint to create the
        // constrained state space.
        switch (type)
        {
            case PJ:
                OMPL_INFORM("Using Projection-Based State Space!");
                css = std::make_shared<ob::ProjectedStateSpace>(space, constraint);
                csi = std::make_shared<ob::ConstrainedSpaceInformation>(css);
                break;
            case AT:
                OMPL_INFORM("Using Atlas-Based State Space!");
                css = std::make_shared<ob::AtlasStateSpace>(space, constraint);
                csi = std::make_shared<ob::ConstrainedSpaceInformation>(css);
                break;
            case TB:
                OMPL_INFORM("Using Tangent Bundle-Based State Space!");
                css = std::make_shared<ob::TangentBundleStateSpace>(space, constraint);
                csi = std::make_shared<ob::TangentBundleSpaceInformation>(css);
                break;
        }

        ss = std::make_shared<og::SimpleSetup>(csi);
    }

    void setConstrainedOptions(struct ConstrainedOptions &opt)
    {
        c_opt = opt;

        constraint->setTolerance(opt.tolerance);
        constraint->setMaxIterations(opt.tries);

        css->setDelta(opt.delta);
    }

    void setAtlasOptions(struct AtlasOptions &opt)
    {
        a_opt = opt;

        if (!(type == AT || type == TB))
            return;

        auto &&atlas = css->as<ob::AtlasStateSpace>();
        atlas->setExploration(opt.exploration);
        atlas->setEpsilon(opt.epsilon);
        atlas->setRho(opt.rho);
        atlas->setAlpha(opt.alpha);
        atlas->setLambda(opt.lambda);
        atlas->setMaxChartsPerExtension(opt.charts);
        atlas->setBias(opt.bias);
        atlas->setSeparated(!opt.separate);

        atlas->setup();
    }

    void setStartAndGoalStates(const Eigen::Ref<const Eigen::VectorXd> &start,
                               const Eigen::Ref<const Eigen::VectorXd> &goal)
    {
        // Create start and goal states (poles of the sphere)
        ob::ScopedState<> sstart(css);
        ob::ScopedState<> sgoal(css);

        sstart->as<ob::ProjectedStateSpace::StateType>()->vectorView() = start;
        sgoal->as<ob::ProjectedStateSpace::StateType>()->vectorView() = goal;

        switch (type)
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

    void setPlanner(enum PLANNER_TYPE planner, const std::string &projection = "")
    {
        const bool isProj = projection != "";

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
                if (type == AT || type == TB)
                    kpiece->setRange(css->as<ob::AtlasStateSpace>()->getRho_s());
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

    enum SPACE_TYPE type;

    ob::StateSpacePtr space;
    ob::ConstraintPtr constraint;

    ob::ConstrainedStateSpacePtr css;
    ob::ConstrainedSpaceInformationPtr csi;

    ob::PlannerPtr pp;

    og::SimpleSetupPtr ss;

    struct ConstrainedOptions c_opt;
    struct AtlasOptions a_opt;
};

#endif
