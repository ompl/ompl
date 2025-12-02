/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, Queen's University
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
 *   * Neither the name of the Queen's University nor the names of its
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

/* Author: Tyler Wilson */

#include "ompl/geometric/planners/rrt/AORRTC.h"
#include <limits>
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"

ompl::geometric::AORRTC::AORRTC(const base::SpaceInformationPtr &si)
  : base::Planner(si, "AORRTC")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;
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
}

ompl::base::Cost ompl::geometric::AORRTC::bestCost() const
{
    return bestCost_;
}

void ompl::geometric::AORRTC::setup()
{
    Planner::setup();

    aox_planner = std::make_shared<ompl::geometric::AOXRRTConnect>(si_);
    aox_planner->setProblemDefinition(pdef_);
    aox_planner->setName("AOXRRTConnect");
    aox_planner->setRange(getRange());

    aox_planner->setup();

    psk_ = std::make_shared<PathSimplifier>(si_);
    initCost_ = std::numeric_limits<double>::infinity();
    solve_status = base::PlannerStatus::TIMEOUT;
}

/* Reset our planner to begin a new search */
void ompl::geometric::AORRTC::reset(bool solvedProblem)
{
    aox_planner->clearQuery();
    aox_planner->setFoundPath(nullptr);
    aox_planner->reset(solvedProblem);
    solve_status = base::PlannerStatus::TIMEOUT;
}

void ompl::geometric::AORRTC::setProblemDefinition(const base::ProblemDefinitionPtr &pdef)
{
    pdef_ = pdef;

    if (aox_planner != nullptr)
    {
        aox_planner->setProblemDefinition(pdef);
    }
}

void ompl::geometric::AORRTC::freeMemory()
{
    aox_planner->clear();
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
        /* If we already have a solution (and therefore a reasonable cost range) */
        if (pdef_->hasExactSolution())
        {
            /* Inform the planner about our cost bound */
            aox_planner->setPathCost(bestCost_.value());
            
            /* Try to solve the planning problem */
            solve_status = aox_planner->solve(ptc);

            if (solve_status == base::PlannerStatus::EXACT_SOLUTION)
            {
                /* If we found a solution, extract it and update our path */
                const base::PathPtr path = aox_planner->getFoundPath();

                /* Update our best path if our new path is better */
                if (path->length() < bestCost_.value())
                {
                    bestPath_ = path;
                    bestCost_ = ompl::base::Cost(path->length());

                    pdef_->addSolutionPath(bestPath_, false, 0.0, getName());
                }

                /* Simplify our solution */
                simplifySolution(path, ptc);

                /* Again, update our best path if our new (simplified) path is better */
                if (path->length() < bestCost_.value())
                {
                    OMPL_INFORM("%s: AOX search simplified to cost %.5f", getName().c_str(), path->length());

                    bestPath_ = path;
                    bestCost_ = ompl::base::Cost(path->length());

                    pdef_->addSolutionPath(bestPath_, false, 0.0, getName());
                }
            }            
        }

        /* Find an initial solution */
        else
        {
            solve_status = aox_planner->solve(ptc);

            /* If we found a solution, extract it and update our path */
            if (solve_status == base::PlannerStatus::EXACT_SOLUTION)
            {
                initCost_ = pdef_->getSolutionPath()->length();
                bestPath_ = pdef_->getSolutionPath();

                bestCost_ = ompl::base::Cost(bestPath_->length());
                pdef_->addSolutionPath(bestPath_, false, 0.0, getName());

                simplifySolution(bestPath_, ptc);
                OMPL_INFORM("%s: Initial search simplified to cost %.5f", getName().c_str(),
                            pdef_->getSolutionPath()->length());

                bestCost_ = ompl::base::Cost(bestPath_->length());
                pdef_->addSolutionPath(bestPath_, false, 0.0, getName());
            }
        }

        /* If we ran into errors, exit early */
        if (solve_status == base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE ||
            solve_status == base::PlannerStatus::INVALID_START ||
            solve_status == base::PlannerStatus::INVALID_GOAL)
        {
            return solve_status;
        }

        /* If we found a solution, we can reset our planner */
        if (solve_status == base::PlannerStatus::EXACT_SOLUTION)
        {
            reset(true);
        }

        /* If we didn't find a solution on this iteration of solve, reset our planner.
        (PDT visualize calls solve for each iteration of the search,
        so we need to check if our planner actually needs to reset) */
        else if (aox_planner->internalResetCondition())
        {
            OMPL_INFORM("%s: Restarted before finding a solution", getName().c_str());
            reset(false);
        }

        /* Something has surely gone wrong if you have found a zero-length path */
        if (bestCost_.value() == 0)
        {
            OMPL_ERROR("%s: Zero-length path found. May have a common start/goal.", getName().c_str());
            return base::PlannerStatus::INVALID_GOAL;
        }
    } while (!ptc);

    /* Now that we're done, return whether or not we timed out */
    if (pdef_->hasExactSolution())
    {
        return base::PlannerStatus::EXACT_SOLUTION;
    } else {
        return base::PlannerStatus::TIMEOUT;
    }
}

void ompl::geometric::AORRTC::getPlannerData(base::PlannerData &data) const
{
    aox_planner->getPlannerData(data);
}
