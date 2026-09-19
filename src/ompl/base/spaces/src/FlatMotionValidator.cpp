/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Rice University
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

/* Author: Clayton W. Ramsey */

#include "ompl/base/spaces/FlatMotionValidator.h"

#include "ompl/base/SpaceInformation.h"
#include "ompl/util/Exception.h"

#include <cstdint>
#include <utility>

namespace ompl::base
{
    void FlatMotionValidator::defaultSettings()
    {
        stateSpace_ = dynamic_cast<FlatStateSpace *>(si_->getStateSpace().get());
        if (stateSpace_ == nullptr)
            throw Exception("FlatMotionValidator needs a FlatStateSpace");
    }

    bool FlatMotionValidator::checkMotion(const State *s1, const State *s2) const
    {
        // The motion starts somewhere already known to be valid, so only the far end needs checking
        // before the interior.
        if (!si_->isValid(s2))
        {
            invalid_++;
            return false;
        }

        // One solve for the whole edge.
        // Handing the motion to interpolate with the flag already down keeps every sample off the
        // steering solver.
        std::optional<FlatMotion> motion = stateSpace_->steer(s1, s2);
        bool firstTime = false;
        const unsigned int count = motion.has_value() ? stateSpace_->validSegmentCount(*motion) : 1u;

        bool result = true;
        if (count >= 2u)
        {
            State *test = si_->allocState();

            // Halving the remaining span rather than sweeping it rejects a blocked edge sooner, whichever
            // end the blockage sits near.
            //
            // Starting the stride at a power of two at or above the count and halving it down to 2 walks
            // all of them coarse to fine, each exactly once, with no work list to hold them in.
            std::uint64_t stride = 1u;
            while (stride < count)
                stride *= 2u;

            for (; result && stride > 1u; stride /= 2u)
            {
                for (std::uint64_t sample = stride / 2u; sample < count; sample += stride)
                {
                    stateSpace_->interpolate(s1, s2, static_cast<double>(sample) / count, firstTime, motion, test);
                    if (!si_->isValid(test))
                    {
                        result = false;
                        break;
                    }
                }
            }

            si_->freeState(test);
        }

        if (result)
            valid_++;
        else
            invalid_++;
        return result;
    }

    bool FlatMotionValidator::checkMotion(const State *s1, const State *s2, std::pair<State *, double> &lastValid) const
    {
        std::optional<FlatMotion> motion = stateSpace_->steer(s1, s2);
        bool firstTime = false;
        const unsigned int count = motion.has_value() ? stateSpace_->validSegmentCount(*motion) : 1u;

        bool result = true;
        if (count >= 2u)
        {
            State *test = si_->allocState();

            // Walking in order rather than bisecting, because the caller asked where the motion stopped
            // being valid and only the first invalid sample answers that.
            for (unsigned int sample = 1u; sample < count; ++sample)
            {
                stateSpace_->interpolate(s1, s2, static_cast<double>(sample) / count, firstTime, motion, test);
                if (!si_->isValid(test))
                {
                    lastValid.second = static_cast<double>(sample - 1u) / count;
                    if (lastValid.first != nullptr)
                        stateSpace_->interpolate(s1, s2, lastValid.second, firstTime, motion, lastValid.first);
                    result = false;
                    break;
                }
            }

            si_->freeState(test);
        }

        if (result && !si_->isValid(s2))
        {
            lastValid.second = static_cast<double>(count - 1u) / count;
            if (lastValid.first != nullptr)
                stateSpace_->interpolate(s1, s2, lastValid.second, firstTime, motion, lastValid.first);
            result = false;
        }

        if (result)
            valid_++;
        else
            invalid_++;
        return result;
    }
}  // namespace ompl::base
