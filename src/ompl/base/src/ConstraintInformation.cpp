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

#include "ompl/base/ConstraintInformation.h"

ompl::base::ConstraintInformation::ConstraintInformation()
{
}

ompl::base::ConstraintInformation::~ConstraintInformation()
{
}

void ompl::base::ConstraintInformation::addConstraint(const ConstraintPtr &constraint)
{
    constraints_.push_back(constraint);
}

unsigned int ompl::base::ConstraintInformation::numConstraints() const
{
    return constraints_.size();
}

ompl::base::ConstraintPtr ompl::base::ConstraintInformation::getConstraint(unsigned int idx) const
{
    assert(idx < constraints_.size());
    return constraints_[idx];
}

bool ompl::base::ConstraintInformation::isSatisfied(const State *state) const
{
    bool valid = true;
    for (size_t i = 0; i < constraints_.size() && valid; ++i)
        valid = constraints_[i]->isSatisfied(state);
    return valid;
}

bool ompl::base::ConstraintInformation::sample(State *state) const
{
    bool valid = true;
    for (size_t i = 0; i < constraints_.size() && valid; ++i)
        valid = constraints_[i]->sample(state);
    return valid;
}

bool ompl::base::ConstraintInformation::project(State *state) const
{
    bool valid = true;
    for (size_t i = 0; i < constraints_.size() && valid; ++i)
        valid = constraints_[i]->project(state);
    return valid;
}