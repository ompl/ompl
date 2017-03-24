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

/* Author: Matt Maly */

#include "ompl/control/planners/syclop/SyclopEST.h"
#include "ompl/base/goals/GoalSampleableRegion.h"

void ompl::control::SyclopEST::setup()
{
    Syclop::setup();
    sampler_ = si_->allocStateSampler();
    controlSampler_ = siC_->allocControlSampler();
    lastGoalMotion_ = nullptr;
}

void ompl::control::SyclopEST::clear()
{
    Syclop::clear();
    freeMemory();
    motions_.clear();
    lastGoalMotion_ = nullptr;
}

void ompl::control::SyclopEST::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    double delta = siC_->getPropagationStepSize();

    if (lastGoalMotion_ != nullptr)
        data.addGoalVertex(lastGoalMotion_->state);

    for (auto motion : motions_)
    {
        if (motion->parent != nullptr)
        {
            if (data.hasControls())
                data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state),
                             control::PlannerDataEdgeControl(motion->control, motion->steps * delta));
            else
                data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
        }
        else
            data.addStartVertex(base::PlannerDataVertex(motion->state));
    }
}

ompl::control::Syclop::Motion *ompl::control::SyclopEST::addRoot(const base::State *s)
{
    auto *motion = new Motion(siC_);
    si_->copyState(motion->state, s);
    siC_->nullControl(motion->control);
    motions_.push_back(motion);
    return motion;
}

void ompl::control::SyclopEST::selectAndExtend(Region &region, std::vector<Motion *> &newMotions)
{
    Motion *treeMotion = region.motions[rng_.uniformInt(0, region.motions.size() - 1)];
    Control *rctrl = siC_->allocControl();
    base::State *newState = si_->allocState();

    controlSampler_->sample(rctrl, treeMotion->state);
    unsigned int duration =
        controlSampler_->sampleStepCount(siC_->getMinControlDuration(), siC_->getMaxControlDuration());
    duration = siC_->propagateWhileValid(treeMotion->state, rctrl, duration, newState);

    if (duration >= siC_->getMinControlDuration())
    {
        auto *motion = new Motion(siC_);
        si_->copyState(motion->state, newState);
        siC_->copyControl(motion->control, rctrl);
        motion->steps = duration;
        motion->parent = treeMotion;
        motions_.push_back(motion);
        newMotions.push_back(motion);

        lastGoalMotion_ = motion;
    }

    siC_->freeControl(rctrl);
    si_->freeState(newState);
}

void ompl::control::SyclopEST::freeMemory()
{
    for (auto m : motions_)
    {
        if (m->state != nullptr)
            si_->freeState(m->state);
        if (m->control != nullptr)
            siC_->freeControl(m->control);
        delete m;
    }
}
