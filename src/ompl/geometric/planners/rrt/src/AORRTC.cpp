/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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

/* Author: Ioan Sucan */

#include "ompl/geometric/planners/rrt/AORRTC.h"
#include <limits>
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"

ompl::geometric::AORRTC::AORRTC(const base::SpaceInformationPtr &si, bool addIntermediateStates)
  : base::Planner(si, "AORRTC")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    addIntermediateStates_ = addIntermediateStates;
}

ompl::geometric::AORRTC::~AORRTC()
{
    freeMemory();
}

void ompl::geometric::AORRTC::clear()
{
    setup_ = false;
    Planner::clear();
    freeMemory();

    init_planner->clear();
    aox_planner->clear();
}

ompl::base::Cost ompl::geometric::AORRTC::bestCost() const
{
    return bestCost_;
}

void ompl::geometric::AORRTC::setup()
{
    Planner::setup();
    init_planner = std::make_shared<ompl::geometric::RRTConnect>(si_);
    init_planner->setProblemDefinition(pdef_);
    init_planner->setName("Initial RRTConnect");
    init_planner->setRange(getRange());
    init_planner->setIntermediateStates(getIntermediateStates());

    aox_planner = std::make_shared<ompl::geometric::AOXRRTConnect>(si_);
    aox_planner->setProblemDefinition(pdef_);
    aox_planner->setName("AOXRRTConnect");
    aox_planner->setRange(getRange());
    aox_planner->setIntermediateStates(getIntermediateStates());

    init_planner->setup();
    aox_planner->setup();

    psk_ = std::make_shared<PathSimplifier>(si_);

    initCost_ = std::numeric_limits<double>::infinity();

    solve_status = base::PlannerStatus::TIMEOUT;
    aox_solve_status = base::PlannerStatus::TIMEOUT;
}

void ompl::geometric::AORRTC::setProblemDefinition(const base::ProblemDefinitionPtr &pdef)
{
    pdef_ = pdef;

    if (init_planner != nullptr)
    {
        init_planner->setProblemDefinition(pdef);
        aox_planner->setProblemDefinition(pdef);
    }
}

void ompl::geometric::AORRTC::freeMemory()
{
    aox_planner->freeMemoryPublic();
}

void ompl::geometric::AORRTC::simplifySolution(const base::PathPtr &p, const base::PlannerTerminationCondition &ptc)
{
    if (pdef_)
    {
        if (p)
        {
            auto &path = static_cast<PathGeometric &>(*p);
            psk_->simplify(path, ptc);
            return;
        }
    }
    OMPL_WARN("No solution to simplify");
}

ompl::base::PlannerStatus ompl::geometric::AORRTC::solve(const base::PlannerTerminationCondition &ptc)
{
    do
    {
        if (pdef_->hasExactSolution())
        {
            aox_planner->setPathCost(bestCost_.value());

            OMPL_INFORM("%s: Best cost is %.5f", getName().c_str(), bestCost_);
            aox_solve_status = aox_planner->solve(ptc);

            if (aox_solve_status != base::PlannerStatus::EXACT_SOLUTION)
            {
                OMPL_INFORM("%s: Restarted before finding a solution", getName().c_str());
            }
            else
            {
                const base::PathPtr path = aox_planner->getFoundPath();

                // OMPL_INFORM("%s: AOX search found a solution with cost %.5f. Simplifying...", getName().c_str(),
                //             path->length());
                simplifySolution(path, ptc);
                OMPL_INFORM("%s: AOX search simplified to cost %.5f", getName().c_str(), path->length());

                if (path->length() < bestCost_.value())
                {
                    bestPath_ = path;
                    bestCost_ = ompl::base::Cost(path->length());

                    pdef_->addSolutionPath(bestPath_, false, 0.0, getName());
                }

                aox_planner->clearQuery();
                aox_planner->setProblemDefinition(pdef_);
            }

            aox_planner->setFoundPath(nullptr);
            aox_solve_status = base::PlannerStatus::TIMEOUT;
        }

        else
        {
            solve_status = init_planner->solve(ptc);

            if (solve_status != base::PlannerStatus::EXACT_SOLUTION)
            {
                OMPL_INFORM("%s: Initial search restarted before finding a solution", getName().c_str());
            }
            else
            {
                initCost_ = pdef_->getSolutionPath()->length();
                bestPath_ = pdef_->getSolutionPath();

                // OMPL_INFORM("%s: Initial search found a solution with cost %.5f. Simplifying...", getName().c_str(),
                //             pdef_->getSolutionPath()->length());
                simplifySolution(bestPath_, ptc);
                OMPL_INFORM("%s: Initial search simplified to cost %.5f", getName().c_str(),
                            pdef_->getSolutionPath()->length());

                bestCost_ = ompl::base::Cost(bestPath_->length());

                pdef_->addSolutionPath(bestPath_, false, 0.0, getName());
            }
        }
    } while (!ptc);

    return solve_status;
}

void ompl::geometric::AORRTC::getPlannerData(base::PlannerData &data) const
{
    if (pdef_->hasExactSolution())
    {
        aox_planner->getPlannerData(data);
    }
    else
    {
        init_planner->getPlannerData(data);
    }
}
