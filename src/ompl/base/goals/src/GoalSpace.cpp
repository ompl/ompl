/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2019, University of Oxford.
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
*   * Neither the name of the University of Oxford nor the names of its
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

/* Author: Jonathan Gammell */

#include "ompl/base/goals/GoalSpace.h"
#include <limits>

ompl::base::GoalSpace::~GoalSpace()
{
}

void ompl::base::GoalSpace::sampleGoal(base::State *st) const
{
    goalSampler_->sampleUniform(st);
}

unsigned int ompl::base::GoalSpace::maxSampleCount() const
{
    return std::numeric_limits<unsigned int>::max();
}

double ompl::base::GoalSpace::distanceGoal(const State *st) const
{
    // Short circuit distance calculation if satisfied.
    if (goalSpace_->satisfiesBounds(st))
    {
        return 0.0;
    }
    // Else, do the work

    // Allocate a new state as a copy of the query
    State *proj = si_->allocState();
    si_->copyState(proj, st);

    // Bring the copy into the bounds of the goal
    goalSpace_->enforceBounds(proj);

    // Calculate the distance between those two states
    double dist = si_->distance(st, proj);

    // Free the copied projection
    si_->freeState(proj);

    // Return the distance
    return dist;
}

void ompl::base::GoalSpace::print(std::ostream &out) const
{
    out << "Goal space, threshold = " << threshold_ << ", memory address = " << this << ", volume = " << goalSpace_->getMeasure() << std::endl;
}

void ompl::base::GoalSpace::setSpace(const StateSpacePtr space)
{
    if (space->getType() != si_->getStateSpace()->getType())
    {
        OMPL_ERROR("GoalSpace: The specified goal space must be the same type as the problem space.");
    }

    goalSpace_ = space;
    goalSampler_ = goalSpace_->allocStateSampler();
}

ompl::base::StateSpacePtr ompl::base::GoalSpace::getSpace() const
{
    return goalSpace_;
}
