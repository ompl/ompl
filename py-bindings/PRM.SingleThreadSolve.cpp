/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Rice University
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

/* Author: Ioan Sucan, James D. Marble */

#include "ompl/base/GoalSampleableRegion.h"

ompl::base::PlannerStatus PRM_wrapper::solve(double solveTime)
{
    using namespace ompl;

    checkValidity();

    static const unsigned int MAX_RANDOM_BOUNCE_STEPS   = 5;
    base::PlannerTerminationCondition ptc = solveTime < 1.0
        ? base::timedPlannerTerminationCondition(solveTime)
        : base::timedPlannerTerminationCondition(solveTime, std::min(solveTime / 100.0, 0.1));
    base::GoalSampleableRegion *goal = dynamic_cast<base::GoalSampleableRegion*>(pdef_->getGoal().get());

    if (!goal)
    {
        msg_.error("Goal undefined or unknown type of goal");
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    // Add the valid start states as milestones
    while (const base::State *st = pis_.nextStart())
        startM_.push_back(addMilestone(si_->cloneState(st)));

    if (startM_.size() == 0)
    {
        msg_.error("There are no valid initial states!");
        return base::PlannerStatus::INVALID_START;
    }

    if (!goal->couldSample())
    {
        msg_.error("Insufficient states in sampleable goal region");
        return base::PlannerStatus::INVALID_GOAL;
    }

    // Ensure there is at least one valid goal state
    if (goal->maxSampleCount() > goalM_.size() || goalM_.empty())
    {
        const base::State *st = goalM_.empty() ? pis_.nextGoal(ptc) : pis_.nextGoal();
        if (st)
            goalM_.push_back(addMilestone(si_->cloneState(st)));

        if (goalM_.empty())
        {
            msg_.error("Unable to find any valid goal states");
            return base::PlannerStatus::INVALID_GOAL;
        }
    }

    if (!sampler_)
        sampler_ = si_->allocValidStateSampler();
    if (!simpleSampler_)
        simpleSampler_ = si_->allocStateSampler();

    unsigned int nrStartStates = boost::num_vertices(g_);
    msg_.inform("Starting with %u states", nrStartStates);

    std::vector<base::State*> xstates(MAX_RANDOM_BOUNCE_STEPS);
    si_->allocStates(xstates);
    bool grow = true;

    // Reset addedSolution_ member
    addedSolution_ = false;
    base::PathPtr sln;
    sln.reset();

    double roadmap_build_time = 0.05;
    while (ptc() == false && !addedSolution_)
    {
        // Check for any new goal states
        if (goal->maxSampleCount() > goalM_.size())
        {
            const base::State *st = pis_.nextGoal();
            if (st)
                goalM_.push_back(addMilestone(si_->cloneState(st)));
        }

        // maintain a 2:1 ratio for growing/expansion of roadmap
        // call growRoadmap() twice as long for every call of expandRoadmap()
        if (grow)
            ompl::geometric::PRM::growRoadmap(base::PlannerOrTerminationCondition(ptc, base::timedPlannerTerminationCondition(2.0*roadmap_build_time)), xstates[0]);
        else
            ompl::geometric::PRM::expandRoadmap(base::PlannerOrTerminationCondition(ptc, base::timedPlannerTerminationCondition(roadmap_build_time)), xstates);
        grow = !grow;

        // Check for a solution
        addedSolution_ = haveSolution (startM_, goalM_, sln);
    }

    msg_.inform("Created %u states", boost::num_vertices(g_) - nrStartStates);

    if (sln)
    {
        if(addedSolution_)
            goal->addSolutionPath (sln);
        else
            // the solution is exact, but not as short as we'd like it to be
            goal->addSolutionPath (sln, true, 0.0);
    }

    si_->freeStates(xstates);

    // Return true if any solution was found.
    return sln ? (addedSolution_ ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::APPROXIMATE_SOLUTION) : base::PlannerStatus::TIMEOUT;
}
