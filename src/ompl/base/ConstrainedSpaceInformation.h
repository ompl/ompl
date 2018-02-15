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

/* Author: Zachary Kingston */

#ifndef OMPL_BASE_CONSTRAINED_SPACE_INFORMATION_
#define OMPL_BASE_CONSTRAINED_SPACE_INFORMATION_

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/spaces/constraint/ConstrainedStateSpace.h"
#include "ompl/base/spaces/constraint/AtlasChart.h"
#include "ompl/base/spaces/constraint/AtlasStateSpace.h"

#include "ompl/util/ClassForward.h"
#include "ompl/util/Console.h"
#include "ompl/util/Exception.h"

namespace ompl
{
    namespace base
    {
        OMPL_CLASS_FORWARD(ConstrainedSpaceInformation);

        class ConstrainedSpaceInformation : public SpaceInformation
        {
        public:
            ConstrainedSpaceInformation(StateSpacePtr space) : SpaceInformation(space)
            {
                stateSpace_->as<ConstrainedStateSpace>()->setSpaceInformation(this);
            }

            unsigned int getMotionStates(const State *s1, const State *s2, std::vector<State *> &states,
                                         unsigned int count, bool endpoints, bool alloc) const override
            {

                bool success = stateSpace_->as<ConstrainedStateSpace>()->traverseManifold(s1, s2, false, &states);

                if (!success && states.size() == 0)
                    states.push_back(cloneState(s1));

                return states.size();
            }
        };

        class TangentBundleSpaceInformation : public ConstrainedSpaceInformation
        {
        public:
            TangentBundleSpaceInformation(StateSpacePtr space) : ConstrainedSpaceInformation(space)
            {
            }

            unsigned int getMotionStates(const State *s1, const State *s2, std::vector<State *> &states,
                                         unsigned int count, bool endpoints, bool alloc) const override
            {
                auto &&atlas = stateSpace_->as<TangentBundleStateSpace>();
                bool success = atlas->traverseManifold(s1, s2, false, &states);

                if (!success && states.size() == 0)
                    states.push_back(cloneState(s1));

                auto it = states.begin();
                for (; it != states.end(); ++it)
                {
                    auto stateT = (*it)->as<AtlasStateSpace::StateType>();
                    Eigen::VectorXd u(atlas->getManifoldDimension());
                    AtlasChart *c = atlas->getChart(stateT);
                    c->psiInverse(stateT->constVectorView(), u);

                    if (!c->psi(u, stateT->vectorView()))
                        break;
                }

                while (it != states.end())
                {
                    freeState(*it);
                    it = states.erase(it);
                }

                return states.size();
            }

            bool checkMotion(const State *s1, const State *s2, std::pair<State *, double> &lastValid) const override
            {
                auto &&atlas = stateSpace_->as<TangentBundleStateSpace>();
                bool valid = motionValidator_->checkMotion(s1, s2, lastValid);

                if (lastValid.first)
                {
                    auto stateT = lastValid.first->as<AtlasStateSpace::StateType>();

                    Eigen::VectorXd u(atlas->getManifoldDimension());
                    AtlasChart *c = atlas->getChart(stateT);
                    c->psiInverse(stateT->constVectorView(), u);

                    if (!c->psi(u, stateT->vectorView()))
                        valid = false;
                }

                return valid;
            }
        };
    }
}

#endif
