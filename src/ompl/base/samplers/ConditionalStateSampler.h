/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2021, Technische Universität Berlin (TU Berlin)
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
*   * Neither the name of the TU Berlin nor the names of its
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

/* Author: Francesco Grothe */

#ifndef OMPL_CONDITIONALSTATESAMPLER_H
#define OMPL_CONDITIONALSTATESAMPLER_H

#include "ompl/base/ValidStateSampler.h"
#include "ompl/base/StateSampler.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/spaces/SpaceTimeStateSpace.h"

namespace ompl
{
    namespace base
    {
        /** \brief Representation of a motion */
        class Motion
        {
        public:
            Motion() = default;

            explicit Motion(const SpaceInformationPtr &si) : state(si->allocState()) {}

            ~Motion() = default;

            const base::State *root{nullptr};
            base::State *state{nullptr};
            Motion *parent{nullptr};
            /** \brief The set of motions descending from the current motion */
            std::vector<Motion *> children{};
            // only used by goal tree
            Motion *connectionPoint{nullptr};  // the start tree motion, if there is a direct connection
            int numConnections{0};  // number of connections to the start tree of self and all descendants
        };

        /** \brief The Conditional Sampler samples feasible Space-Time States. First, a space configuration is
         * sampled. Then, a feasible time conditioned on the start and goal states is sampled for the space configuration.*/
        class ConditionalStateSampler : public ValidStateSampler
        {
        public:
            /** \brief The constructor. */
            ConditionalStateSampler(const SpaceInformation *si, Motion *&startMotion,
                                    std::vector<Motion *> &goalMotions, std::vector<Motion *> &newBatchGoalMotions,
                                    bool &sampleOldBatch);

            bool sample(State *state) override;
            bool sampleNear(State *state, const State *near, double distance) override;

        private:
            /** \brief The internal state sampler, that samples the space (but not time). */
            StateSamplerPtr internalSampler_ = si_->allocStateSampler();

            /** \brief References to the start state and goal states. */
            Motion *&startMotion_;

            /** \brief The goal motions of the old batch for time conditioning. */
            std::vector<Motion *> &goalMotions_;

            /** \brief The goal motions of the new batch for time conditioning. */
            std::vector<Motion *> &newBatchGoalMotions_;

            /** \brief References to whether the old or new batch region is sampled. */
            bool &sampleOldBatch_;

            /** \brief Maximum tries to sample a new state, if no new state could be sampled, force a new goal
                 * sample. */
            int maxTries_ = 10;

            /** \brief The random number generator. */
            ompl::RNG rng_;
        };
    }
}



#endif  // OMPL_CONDITIONALSTATESAMPLER_H
