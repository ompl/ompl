/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rice University
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
*   * Neither the name of Rice University nor the names of its
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

#include <queue>
#include <ompl/util/Exception.h>
#include "LinearMotionValidator.h"


LinearMotionValidator::LinearMotionValidator(ompl::base::SpaceInformation *si) : ompl::base::MotionValidator(si)
{
    defaultSettings();
}

LinearMotionValidator::LinearMotionValidator(const ompl::base::SpaceInformationPtr& si) : ompl::base::MotionValidator(si)
{
    defaultSettings();
}

LinearMotionValidator::~LinearMotionValidator()
{
}

bool LinearMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const
{
    // assume motion starts in a valid configuration (s1 is valid)
    if (!si_->isValid(s2))
    {
        invalid_++;
        return false;
    }

    bool result = true;
    int nd = ss_->validSegmentCount(s1, s2);

    if (nd > 1)
    {
        /* temporary storage for the checked state */
        ompl::base::State *test = si_->allocState();

        for (int j = 1; j < nd && result; ++j)
        {
            ss_->interpolate(s1, s2, (double)j / (double)nd, test);
            result = si_->isValid(test);
        }
        si_->freeState(test);
    }

    if (result)
        valid_++;
    else
        invalid_++;

    return result;
}

bool LinearMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2,
                                        std::pair<ompl::base::State*, double> &lastValid) const
{
    // assume motion starts in a valid configuration (s1 is valid)
    bool result = true;
    int nd = ss_->validSegmentCount(s1, s2);

    if (nd > 1)
    {
        // temporary storage for the checked state
        ompl::base::State *test = si_->allocState();

        for (int j = 1 ; j < nd ; ++j)
        {
            ss_->interpolate(s1, s2, (double)j / (double)nd, test);
            if (!si_->isValid(test))
            {
                lastValid.second = (double)(j - 1) / (double)nd;
                if (lastValid.first)
                    ss_->interpolate(s1, s2, lastValid.second, lastValid.first);
                result = false;
                break;
            }
        }
        si_->freeState(test);
    }

    if (result)
        if (!si_->isValid(s2))
        {
            lastValid.second = (double)(nd - 1) / (double)nd;
            if (lastValid.first)
                ss_->interpolate(s1, s2, lastValid.second, lastValid.first);
            result = false;
        }

    if (result)
        valid_++;
    else
        invalid_++;

    return result;
}

void LinearMotionValidator::defaultSettings()
{
    ss_ = si_->getStateSpace().get();
    if (!ss_)
        throw ompl::Exception("No state space for motion validator");
}