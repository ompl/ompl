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

#include "ompl/control/planners/syclop/SyclopRRT.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"

void ompl::control::SyclopRRT::setup()
{
    Syclop::setup();
    sampler_ = si_->allocStateSampler();
    controlSampler_ = siC_->allocDirectedControlSampler();
    lastGoalMotion_ = nullptr;

    // Create a default GNAT nearest neighbors structure if the user doesn't want
    // the default regionalNN check from the discretization
    if (!nn_ && !regionalNN_)
    {
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
        nn_->setDistanceFunction([this](Motion *a, const Motion *b)
                                 {
                                     return distanceFunction(a, b);
                                 });
    }
}

void ompl::control::SyclopRRT::clear()
{
    Syclop::clear();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
}

void ompl::control::SyclopRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);
    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);
    double delta = siC_->getPropagationStepSize();

    if (lastGoalMotion_ != nullptr)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto &motion : motions)
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

ompl::control::Syclop::Motion *ompl::control::SyclopRRT::addRoot(const base::State *s)
{
    auto *motion = new Motion(siC_);
    si_->copyState(motion->state, s);
    siC_->nullControl(motion->control);

    if (nn_)
        nn_->add(motion);
    return motion;
}

void ompl::control::SyclopRRT::selectAndExtend(Region &region, std::vector<Motion *> &newMotions)
{
    auto *rmotion = new Motion(siC_);
    base::StateSamplerPtr sampler(si_->allocStateSampler());
    std::vector<double> coord(decomp_->getDimension());
    decomp_->sampleFromRegion(region.index, rng_, coord);
    decomp_->sampleFullState(sampler, coord, rmotion->state);

    Motion *nmotion;
    if (regionalNN_)
    {
        /* Instead of querying the nearest neighbors datastructure over the entire tree of motions,
         * here we perform a linear search over all motions in the selected region and its neighbors. */
        std::vector<int> searchRegions;
        decomp_->getNeighbors(region.index, searchRegions);
        searchRegions.push_back(region.index);

        std::vector<Motion *> motions;
        for (const auto &i : searchRegions)
        {
            const std::vector<Motion *> &regionMotions = getRegionFromIndex(i).motions;
            motions.insert(motions.end(), regionMotions.begin(), regionMotions.end());
        }

        std::vector<Motion *>::const_iterator i = motions.begin();
        nmotion = *i;
        double minDistance = distanceFunction(rmotion, nmotion);
        ++i;
        while (i != motions.end())
        {
            Motion *m = *i;
            const double dist = distanceFunction(rmotion, m);
            if (dist < minDistance)
            {
                nmotion = m;
                minDistance = dist;
            }
            ++i;
        }
    }
    else
    {
        assert(nn_);
        nmotion = nn_->nearest(rmotion);
    }

    unsigned int duration =
        controlSampler_->sampleTo(rmotion->control, nmotion->control, nmotion->state, rmotion->state);
    if (duration >= siC_->getMinControlDuration())
    {
        rmotion->steps = duration;
        rmotion->parent = nmotion;
        newMotions.push_back(rmotion);
        if (nn_)
            nn_->add(rmotion);
        lastGoalMotion_ = rmotion;
    }
    else
    {
        si_->freeState(rmotion->state);
        siC_->freeControl(rmotion->control);
        delete rmotion;
    }
}

void ompl::control::SyclopRRT::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto m : motions)
        {
            if (m->state != nullptr)
                si_->freeState(m->state);
            if (m->control != nullptr)
                siC_->freeControl(m->control);
            delete m;
        }
    }
}
