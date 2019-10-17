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

#include <utility>

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/spaces/constraint/ConstrainedStateSpace.h"
#include "ompl/base/spaces/constraint/AtlasChart.h"
#include "ompl/base/spaces/constraint/AtlasStateSpace.h"
#include "ompl/base/spaces/constraint/TangentBundleStateSpace.h"

#include "ompl/util/ClassForward.h"
#include "ompl/util/Console.h"
#include "ompl/util/Exception.h"

namespace ompl
{
    namespace base
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::ConstrainedSpaceInformation */
        OMPL_CLASS_FORWARD(ConstrainedSpaceInformation);
        /// @endcond

        /** \brief Valid state sampler for constrained state spaces. */
        class ConstrainedValidStateSampler : public ValidStateSampler
        {
        public:
            /** \brief Constructor. Create a valid state sampler for a
             * constrained state space. */
            ConstrainedValidStateSampler(const SpaceInformation *si)
              : ValidStateSampler(si)
              , sampler_(si->getStateSpace()->allocStateSampler())
              , constraint_(si->getStateSpace()->as<ompl::base::ConstrainedStateSpace>()->getConstraint())
            {
            }

            bool sample(State *state) override
            {
                // Rejection sample for at most attempts_ tries.
                unsigned int tries = 0;
                bool valid;

                do
                    sampler_->sampleUniform(state);
                while (!(valid = si_->isValid(state) && constraint_->isSatisfied(state)) && ++tries < attempts_);

                return valid;
            }

            bool sampleNear(State *state, const State *near, double distance) override
            {
                // Rejection sample for at most attempts_ tries.
                unsigned int tries = 0;
                bool valid;
                do
                    sampler_->sampleUniformNear(state, near, distance);
                while (!(valid = si_->isValid(state) && constraint_->isSatisfied(state)) && ++tries < attempts_);

                return valid;
            }

        private:
            /** \brief Underlying constrained state sampler. */
            StateSamplerPtr sampler_;

            /** \brief Constraint function. */
            const ConstraintPtr constraint_;
        };

        /** \brief Space information for a constrained state space. Implements
         * more direct for getting motion states. */
        class ConstrainedSpaceInformation : public SpaceInformation
        {
        public:
            /** \brief Constructor. Sets the instance of the state space to plan with. */
            ConstrainedSpaceInformation(StateSpacePtr space) : SpaceInformation(std::move(space))
            {
                stateSpace_->as<ConstrainedStateSpace>()->setSpaceInformation(this);
                setValidStateSamplerAllocator([](const SpaceInformation *si) -> std::shared_ptr<ValidStateSampler> {
                    return std::make_shared<ConstrainedValidStateSampler>(si);
                });
            }

            /** \brief Get \e count states that make up a motion between \e s1
                and \e s2. Returns the number of states that were added to \e
                states. Uses the constrained state space's manifold traversal
                method to obtain states. Will always allocate states.

                Otherwise, fewer states can be returned.
                \param s1 the start state of the considered motion
                \param s2 the end state of the considered motion
                \param states the computed set of states along the specified motion
                \param count is currently ignored
                \param endpoints flag indicating whether \e s1 and \e s2 are to be included in states
                \param alloc is currently ignored */
            unsigned int getMotionStates(const State *s1, const State *s2, std::vector<State *> &states,
                                         unsigned int /*count*/, bool endpoints, bool /*alloc*/) const override
            {
                bool success = stateSpace_->as<ConstrainedStateSpace>()->discreteGeodesic(s1, s2, true, &states);

                if (endpoints)
                {
                    if (!success && states.empty())
                        states.push_back(cloneState(s1));

                    if (success)
                        states.push_back(cloneState(s2));
                }

                return states.size();
            }
        };

        /** \brief Space information for a tangent bundle-based state space.
         * Implements more direct for getting motion states and checking motion,
         * as the lazy approach requires post-processing. */
        class TangentBundleSpaceInformation : public ConstrainedSpaceInformation
        {
        public:
            /** \brief Constructor. Sets the instance of the state space to plan with. */
            TangentBundleSpaceInformation(StateSpacePtr space) : ConstrainedSpaceInformation(std::move(space))
            {
            }

            /** \brief Get \e count states that make up a motion between \e s1
                and \e s2. Returns the number of states that were added to \e
                states. Uses the constrained state space's manifold traversal
                method to obtain states. Will always allocate states. As tangent
                bundle is lazy, the states are projected onto the manifold so
                that they satisfy constraints.

                Otherwise, fewer states can be returned.
                \param s1 the start state of the considered motion
                \param s2 the end state of the considered motion
                \param states the computed set of states along the specified motion
                \param count is currently ignored
                \param endpoints flag indicating whether \e s1 and \e s2 are to be included in states
                \param alloc is currently ignored
            */
            unsigned int getMotionStates(const State *s1, const State *s2, std::vector<State *> &states,
                                         unsigned int /*count*/, bool /*endpoints*/, bool /*alloc*/) const override
            {
                auto &&atlas = stateSpace_->as<TangentBundleStateSpace>();

                std::vector<State *> temp;
                bool success = atlas->discreteGeodesic(s1, s2, true, &temp);

                if (!success && temp.empty())
                    temp.push_back(cloneState(s1));

                auto it = temp.begin();
                for (; it != temp.end(); ++it)
                {
                    auto astate = (*it)->as<AtlasStateSpace::StateType>();
                    if (!atlas->project(astate))
                        break;

                    states.push_back(astate);
                }

                while (it != temp.end())
                    freeState(*it++);

                return states.size();
            }

            /** \brief Incrementally check if the path between two motions is
                valid. Also compute the last state that was valid and the time
                of that state. The time is used to parametrize the motion from
                s1 to s2, s1 being at t = 0 and s2 being at t = 1. This function
                assumes s1 is valid. As tangent bundle is lazy, the last valid
                state is projected onto the manifold.

                \param s1 start state of the motion to be checked (assumed to be valid)
                \param s2 final state of the motion to be checked
                \param lastValid first: storage for the last valid state (may be nullptr); this need not be different
                from \e s1 or \e s2. second: the time (between 0 and 1) of  the last valid state, on the motion from \e
                s1 to \e s2 */
            bool checkMotion(const State *s1, const State *s2, std::pair<State *, double> &lastValid) const override
            {
                auto &&atlas = stateSpace_->as<TangentBundleStateSpace>();
                bool valid = motionValidator_->checkMotion(s1, s2, lastValid);

                if (lastValid.first != nullptr)
                {
                    auto astate = lastValid.first->as<AtlasStateSpace::StateType>();
                    if (!atlas->project(astate))
                        valid = false;
                }

                return valid;
            }
        };
    }
}

#endif
