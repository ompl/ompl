/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Rice University
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

/* Author: Ryan Luna */

#include <algorithm>  // for std::min
#include <boost/algorithm/string.hpp>

#include "ompl/geometric/planners/AnytimePathShortening.h"
#include "ompl/geometric/planners/fmt/BFMT.h"
#include "ompl/geometric/planners/est/BiEST.h"
#include "ompl/geometric/planners/rrt/BiTRRT.h"
#include "ompl/geometric/planners/informedtrees/BITstar.h"
#include "ompl/geometric/planners/kpiece/BKPIECE1.h"
#include "ompl/geometric/planners/est/EST.h"
#include "ompl/geometric/planners/fmt/FMT.h"
#include "ompl/geometric/planners/kpiece/KPIECE1.h"
#include "ompl/geometric/planners/rrt/LazyLBTRRT.h"
#include "ompl/geometric/planners/prm/LazyPRM.h"
#include "ompl/geometric/planners/prm/LazyPRMstar.h"
#include "ompl/geometric/planners/rrt/LazyRRT.h"
#include "ompl/geometric/planners/kpiece/LBKPIECE1.h"
#include "ompl/geometric/planners/rrt/LBTRRT.h"
#include "ompl/geometric/planners/pdst/PDST.h"
#include "ompl/geometric/planners/prm/PRM.h"
#include "ompl/geometric/planners/prm/PRMstar.h"
#include "ompl/geometric/planners/est/ProjEST.h"
#include "ompl/geometric/planners/rrt/RRT.h"
#include "ompl/geometric/planners/rrt/RRTConnect.h"
#include "ompl/geometric/planners/rrt/RRTstar.h"
#include "ompl/geometric/planners/rrt/RRTXstatic.h"
#include "ompl/geometric/planners/sbl/SBL.h"
#include "ompl/geometric/planners/prm/SPARS.h"
#include "ompl/geometric/planners/prm/SPARStwo.h"
#include "ompl/geometric/planners/sst/SST.h"
#include "ompl/geometric/planners/stride/STRIDE.h"
#include "ompl/geometric/planners/rrt/TRRT.h"
#include "ompl/geometric/PathHybridization.h"
#include "ompl/geometric/PathSimplifier.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/util/String.h"

ompl::geometric::AnytimePathShortening::AnytimePathShortening(const ompl::base::SpaceInformationPtr &si)
  : ompl::base::Planner(si, "APS"), defaultNumPlanners_(std::max(1u, std::thread::hardware_concurrency()))
{
    specs_.approximateSolutions = true;
    specs_.multithreaded = true;
    specs_.optimizingPaths = true;

    Planner::declareParam<bool>("shortcut", this, &AnytimePathShortening::setShortcut,
                                &AnytimePathShortening::isShortcutting, "0,1");
    Planner::declareParam<bool>("hybridize", this, &AnytimePathShortening::setHybridize,
                                &AnytimePathShortening::isHybridizing, "0,1");
    Planner::declareParam<unsigned int>("max_hybrid_paths", this, &AnytimePathShortening::setMaxHybridizationPath,
                                        &AnytimePathShortening::maxHybridizationPaths, "0:1:50");
    Planner::declareParam<unsigned int>("num_planners", this, &AnytimePathShortening::setDefaultNumPlanners,
                                        &AnytimePathShortening::getDefaultNumPlanners, "0:64");
    Planner::declareParam<std::string>("planners", this, &AnytimePathShortening::setPlanners,
                                       &AnytimePathShortening::getPlanners);

    addPlannerProgressProperty("best cost REAL", [this] { return getBestCost(); });
}

ompl::geometric::AnytimePathShortening::~AnytimePathShortening() = default;

void ompl::geometric::AnytimePathShortening::addPlanner(base::PlannerPtr &planner)
{
    if (planner && planner->getSpaceInformation().get() != si_.get())
    {
        OMPL_ERROR("NOT adding planner %s: SpaceInformation instances are different", planner->getName().c_str());
        return;
    }

    // Ensure all planners are unique instances
    for (auto &i : planners_)
    {
        if (planner.get() == i.get())
        {
            OMPL_ERROR("NOT adding planner %s: Planner instances MUST be unique", planner->getName().c_str());
            return;
        }
    }

    planners_.push_back(planner);
}

void ompl::geometric::AnytimePathShortening::addPath(const geometric::PathGeometricPtr &path, base::Planner *planner)
{
    const base::OptimizationObjectivePtr &opt(pdef_->getOptimizationObjective());
    base::Cost pathCost = path->cost(opt);
    std::lock_guard<std::mutex> _(lock_);
    if (opt->isCostBetterThan(pathCost, bestCost_))
    {
        bestCost_ = pathCost;
        pdef_->addSolutionPath(path, false, 0.0, planner->getName());
    }
    else if (planner != this)
        pdef_->addSolutionPath(path, false, 0.0, planner->getName());
}

ompl::base::PlannerStatus
ompl::geometric::AnytimePathShortening::solve(const ompl::base::PlannerTerminationCondition &ptc)
{
    std::vector<std::thread> threads;
    geometric::PathHybridization phybrid(si_);

    base::OptimizationObjectivePtr opt = pdef_->getOptimizationObjective();
    if (!opt)
    {
        OMPL_INFORM("%s: No optimization objective specified. Defaulting to optimizing path length for the allowed "
                    "planning time.",
                    getName().c_str());
        opt = std::make_shared<base::PathLengthOptimizationObjective>(si_);
        pdef_->setOptimizationObjective(opt);
    }

    // Disable output from the motion planners, except for errors
    msg::LogLevel currentLogLevel = msg::getLogLevel();
    msg::setLogLevel(std::max(msg::LOG_ERROR, currentLogLevel));

    // Clear any previous planning data for the set of planners
    clear();
    // Spawn a thread for each planner.  This will shortcut the best path after solving.
    for (auto &planner : planners_)
        threads.emplace_back([this, planner, &ptc] { return threadSolve(planner.get(), ptc); });

    geometric::PathSimplifier ps(si_);
    geometric::PathGeometric *sln = nullptr, *prevLastPath = nullptr;
    bestCost_ = opt->infiniteCost();
    std::size_t prevSolCount = 0;
    while (!ptc)
    {
        // We have found a solution that is good enough
        if (opt->isSatisfied(bestCost_))
        {
            ptc.terminate();
            break;
        }

        // Hybridize the set of paths computed. Add the new hybrid path to the mix.
        unsigned int solCount = pdef_->getSolutionCount();
        if (hybridize_ && !ptc && solCount > 1)
        {
            const std::vector<base::PlannerSolution> &paths = pdef_->getSolutions();
            std::size_t numPaths = std::min(solCount, maxHybridPaths_);
            geometric::PathGeometric *lastPath = static_cast<PathGeometric *>(paths[numPaths - 1].path_.get());
            // check if new solution paths have been added to top numPaths paths
            if (lastPath != prevLastPath || (prevSolCount < solCount && solCount <= maxHybridPaths_))
            {
                for (size_t j = 0; j < numPaths && !ptc; ++j)
                    phybrid.recordPath(std::static_pointer_cast<PathGeometric>(paths[j].path_), false);

                phybrid.computeHybridPath();
                sln = phybrid.getHybridPath().get();
                prevLastPath = lastPath;
            }
            else
                sln = static_cast<PathGeometric *>(pdef_->getSolutionPath().get());
            prevSolCount = solCount;
        }
        else if (solCount > 0)
            sln = static_cast<PathGeometric *>(pdef_->getSolutionPath().get());

        if (sln)
        {
            auto pathCopy(std::make_shared<geometric::PathGeometric>(*sln));
            if (shortcut_)  // Shortcut the path
                if (!ps.simplify(*pathCopy, ptc, true))
                    // revert if shortcutting failed
                    pathCopy = std::make_shared<geometric::PathGeometric>(*sln);
            addPath(pathCopy, this);
        }
        if (hybridize_ && phybrid.pathCount() >= maxHybridPaths_)
            phybrid.clear();
    }

    for (auto &thread : threads)
        thread.join();

    msg::setLogLevel(currentLogLevel);
    return pdef_->getSolutionCount() > 0 ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::UNKNOWN;
}

void ompl::geometric::AnytimePathShortening::threadSolve(base::Planner *planner,
                                                         const base::PlannerTerminationCondition &ptc)
{
    // create a local clone of the problem definition where we keep at most one solution
    auto pdef = pdef_->clone();
    geometric::PathSimplifier ps(si_);

    planner->setProblemDefinition(pdef);
    while (!ptc)
    {
        if (planner->solve(ptc) == base::PlannerStatus::EXACT_SOLUTION)
        {
            geometric::PathGeometric *sln = static_cast<geometric::PathGeometric *>(pdef->getSolutionPath().get());
            auto pathCopy(std::make_shared<geometric::PathGeometric>(*sln));
            if (shortcut_)  // Shortcut the path
                ps.shortcutPath(*pathCopy);
            addPath(pathCopy, planner);
        }

        planner->clear();
        pdef->clearSolutionPaths();
    }
}

void ompl::geometric::AnytimePathShortening::clear()
{
    Planner::clear();
    for (auto &planner : planners_)
        planner->clear();
    bestCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());
}

void ompl::geometric::AnytimePathShortening::getPlannerData(ompl::base::PlannerData &data) const
{
    if (planners_.empty())
        return;

    OMPL_WARN("Returning planner data for planner #0");
    getPlannerData(data, 0);
}

void ompl::geometric::AnytimePathShortening::getPlannerData(ompl::base::PlannerData &data, unsigned int idx) const
{
    if (planners_.size() < idx)
        return;
    planners_[idx]->getPlannerData(data);
}

void ompl::geometric::AnytimePathShortening::setup()
{
    Planner::setup();

    if (planners_.empty())
    {
        planners_.reserve(defaultNumPlanners_);
        for (unsigned int i = 0; i < defaultNumPlanners_; ++i)
        {
            planners_.push_back(tools::SelfConfig::getDefaultPlanner(pdef_->getGoal()));
            planners_.back()->setProblemDefinition(pdef_);
        }
        OMPL_INFORM("%s: No planners specified; using %u instances of %s", getName().c_str(), planners_.size(),
                    planners_[0]->getName().c_str());
    }

    for (auto &planner : planners_)
        planner->setup();
}

void ompl::geometric::AnytimePathShortening::checkValidity()
{
    for (auto &planner : planners_)
        planner->checkValidity();
}

unsigned int ompl::geometric::AnytimePathShortening::getNumPlanners() const
{
    return planners_.size();
}

ompl::base::PlannerPtr ompl::geometric::AnytimePathShortening::getPlanner(unsigned int idx) const
{
    assert(idx < planners_.size());
    return planners_[idx];
}

bool ompl::geometric::AnytimePathShortening::isShortcutting() const
{
    return shortcut_;
}

void ompl::geometric::AnytimePathShortening::setShortcut(bool shortcut)
{
    shortcut_ = shortcut;
}

bool ompl::geometric::AnytimePathShortening::isHybridizing() const
{
    return hybridize_;
}

void ompl::geometric::AnytimePathShortening::setHybridize(bool hybridize)
{
    hybridize_ = hybridize;
}

unsigned int ompl::geometric::AnytimePathShortening::maxHybridizationPaths() const
{
    return maxHybridPaths_;
}

void ompl::geometric::AnytimePathShortening::setMaxHybridizationPath(unsigned int maxPathCount)
{
    maxHybridPaths_ = maxPathCount;
}

void ompl::geometric::AnytimePathShortening::setDefaultNumPlanners(unsigned int numPlanners)
{
    defaultNumPlanners_ = numPlanners;
}

unsigned int ompl::geometric::AnytimePathShortening::getDefaultNumPlanners() const
{
    return defaultNumPlanners_;
}

std::string ompl::geometric::AnytimePathShortening::getBestCost() const
{
    return ompl::toString(bestCost_.value());
}

void ompl::geometric::AnytimePathShortening::setPlanners(const std::string &plannerList)
{
    std::vector<std::string> plannerStrings;

    boost::split(plannerStrings, plannerList, boost::is_any_of(","));

    planners_.clear();
    for (const auto &plannerString : plannerStrings)
    {
        std::vector<std::string> plannerAndParams;
        boost::split(plannerAndParams, plannerString, boost::is_any_of("[ ]"));
        const std::string &plannerName = plannerAndParams[0];

        if (plannerName == "BFMT")
            planners_.push_back(std::make_shared<geometric::BFMT>(si_));
        else if (plannerName == "BiEST")
            planners_.push_back(std::make_shared<geometric::BiEST>(si_));
        else if (plannerName == "BiTRRT")
            planners_.push_back(std::make_shared<geometric::BiTRRT>(si_));
        else if (plannerName == "BITstar")
            planners_.push_back(std::make_shared<geometric::BITstar>(si_));
        else if (plannerName == "BKPIECE")
            planners_.push_back(std::make_shared<geometric::BKPIECE1>(si_));
        else if (plannerName == "EST")
            planners_.push_back(std::make_shared<geometric::EST>(si_));
        else if (plannerName == "FMT")
            planners_.push_back(std::make_shared<geometric::FMT>(si_));
        else if (plannerName == "KPIECE")
            planners_.push_back(std::make_shared<geometric::KPIECE1>(si_));
        else if (plannerName == "LazyLBTRRT")
            planners_.push_back(std::make_shared<geometric::LazyLBTRRT>(si_));
        else if (plannerName == "LazyPRM")
            planners_.push_back(std::make_shared<geometric::LazyPRM>(si_));
        else if (plannerName == "LazyPRMstar")
            planners_.push_back(std::make_shared<geometric::LazyPRMstar>(si_));
        else if (plannerName == "LazyRRT")
            planners_.push_back(std::make_shared<geometric::LazyRRT>(si_));
        else if (plannerName == "LBKPIECE")
            planners_.push_back(std::make_shared<geometric::LBKPIECE1>(si_));
        else if (plannerName == "LBTRRT")
            planners_.push_back(std::make_shared<geometric::LBTRRT>(si_));
        else if (plannerName == "PDST")
            planners_.push_back(std::make_shared<geometric::PDST>(si_));
        else if (plannerName == "PRM")
            planners_.push_back(std::make_shared<geometric::PRM>(si_));
        else if (plannerName == "PRMstar")
            planners_.push_back(std::make_shared<geometric::PRMstar>(si_));
        else if (plannerName == "ProjEST")
            planners_.push_back(std::make_shared<geometric::ProjEST>(si_));
        else if (plannerName == "RRT")
            planners_.push_back(std::make_shared<geometric::RRT>(si_));
        else if (plannerName == "RRTConnect")
            planners_.push_back(std::make_shared<geometric::RRTConnect>(si_));
        else if (plannerName == "RRTstar")
            planners_.push_back(std::make_shared<geometric::RRTstar>(si_));
        else if (plannerName == "RRTXstatic")
            planners_.push_back(std::make_shared<geometric::RRTXstatic>(si_));
        else if (plannerName == "SBL")
            planners_.push_back(std::make_shared<geometric::SBL>(si_));
        else if (plannerName == "SPARS")
            planners_.push_back(std::make_shared<geometric::SPARS>(si_));
        else if (plannerName == "SPARStwo")
            planners_.push_back(std::make_shared<geometric::SPARStwo>(si_));
        else if (plannerName == "SST")
            planners_.push_back(std::make_shared<geometric::SST>(si_));
        else if (plannerName == "STRIDE")
            planners_.push_back(std::make_shared<geometric::STRIDE>(si_));
        else if (plannerName == "TRRT")
            planners_.push_back(std::make_shared<geometric::TRRT>(si_));
        else
            OMPL_ERROR("Unknown planner name: %s", plannerName.c_str());

        std::vector<std::string> paramValue;
        for (auto it = plannerAndParams.begin() + 1; it != plannerAndParams.end(); ++it)
            if (!it->empty())
            {
                boost::split(paramValue, *it, boost::is_any_of("="));
                planners_.back()->params().setParam(paramValue[0], paramValue[1]);
            }
    }
}

std::string ompl::geometric::AnytimePathShortening::getPlanners() const
{
    std::stringstream ss;
    for (unsigned int i = 0; i < planners_.size(); ++i)
    {
        if (i > 0)
            ss << ',';
        ss << planners_[i]->getName();

        std::map<std::string, std::string> params;
        planners_[i]->params().getParams(params);
        if (params.size() > 0)
        {
            ss << '[';
            for (auto it = params.begin(); it != params.end(); ++it)
            {
                if (it != params.begin())
                    ss << ' ';
                ss << it->first << '=' << it->second;
            }
            ss << ']';
        }
    }
    return ss.str();
}

void ompl::geometric::AnytimePathShortening::printSettings(std::ostream &out) const
{
    Planner::printSettings(out);
    out << "Settings for planner instances in AnytimePathShortening instance:\n";
    for (const auto &planner : planners_)
    {
        out << "* ";
        planner->printSettings(out);
    }
}
