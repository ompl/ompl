/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, Metron, Inc.
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
 *   * Neither the name of the Metron, Inc. nor the names of its
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

/* Author: Mark Moll */

#ifndef OMPL_BASE_SPACES_DUBINS3D_MOTION_VALIDATOR_
#define OMPL_BASE_SPACES_DUBINS3D_MOTION_VALIDATOR_

#include "ompl/base/SpaceInformation.h"
#include <queue>

namespace ompl::base
{

    /**
     * \brief A 3D Dubins plane motion validator that only uses the state validity checker.
     *  Motions are checked for validity at a specified resolution.
     *
     * This motion validator is almost identical to the DiscreteMotionValidator except that it remembers the optimal
     * PathType between different calls to interpolate. It is intended to be used with the OwenStateSpace,
     * VanaStateSpace, and VanaOwenStateSpace. The SpaceInformation::setMotionValidator method uses this as the default
     * for those state spaces.
     */
    template <class Dubins3DStateSpace>
    class Dubins3DMotionValidator : public MotionValidator
    {
    public:
        Dubins3DMotionValidator(SpaceInformation *si) : MotionValidator(si)
        {
            defaultSettings();
        }
        Dubins3DMotionValidator(const SpaceInformationPtr &si) : MotionValidator(si)
        {
            defaultSettings();
        }
        ~Dubins3DMotionValidator() override = default;
        bool checkMotion(const State *s1, const State *s2) const override
        {
            /* assume motion starts in a valid configuration so s1 is valid */
            if (!si_->isValid(s2))
                return false;
            auto path = stateSpace_->getPath(s1, s2);
            if (!path)
                return false;

            bool result = true;
            int nd = stateSpace_->validSegmentCount(s1, s2);

            /* initialize the queue of test positions */
            std::queue<std::pair<int, int>> pos;
            if (nd >= 2)
            {
                pos.emplace(1, nd - 1);

                /* temporary storage for the checked state */
                State *test = si_->allocState();

                /* repeatedly subdivide the path segment in the middle (and check the middle) */
                while (!pos.empty())
                {
                    std::pair<int, int> x = pos.front();

                    int mid = (x.first + x.second) / 2;
                    stateSpace_->interpolate(s1, s2, (double)mid / (double)nd, *path, test);

                    if (!si_->isValid(test))
                    {
                        result = false;
                        break;
                    }

                    pos.pop();

                    if (x.first < mid)
                        pos.emplace(x.first, mid - 1);
                    if (x.second > mid)
                        pos.emplace(mid + 1, x.second);
                }

                si_->freeState(test);
            }

            if (result)
                valid_++;
            else
                invalid_++;

            return result;
        }
        bool checkMotion(const State *s1, const State *s2, std::pair<State *, double> &lastValid) const override
        {
            auto path = stateSpace_->getPath(s1, s2);
            if (!path)
                return false;

            /* assume motion starts in a valid configuration so s1 is valid */
            bool result = true;
            int nd = stateSpace_->validSegmentCount(s1, s2);

            if (nd > 1)
            {
                /* temporary storage for the checked state */
                State *test = si_->allocState();

                for (int j = 1; j < nd; ++j)
                {
                    stateSpace_->interpolate(s1, s2, (double)j / (double)nd, *path, test);
                    if (!si_->isValid(test))
                    {
                        lastValid.second = (double)(j - 1) / (double)nd;
                        if (lastValid.first != nullptr)
                            stateSpace_->interpolate(s1, s2, lastValid.second, *path, lastValid.first);
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
                    if (lastValid.first != nullptr)
                        stateSpace_->interpolate(s1, s2, lastValid.second, *path, lastValid.first);
                    result = false;
                }

            if (result)
                valid_++;
            else
                invalid_++;

            return result;
        }

    private:
        Dubins3DStateSpace *stateSpace_;
        void defaultSettings()
        {
            stateSpace_ = dynamic_cast<Dubins3DStateSpace *>(si_->getStateSpace().get());
            if (stateSpace_ == nullptr)
                throw Exception("No state space for motion validator");
        }
    };
}  // namespace ompl::base

#endif
