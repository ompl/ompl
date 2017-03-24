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

/* Author: Ioan Sucan */

#ifndef OMPL_BASE_SAMPLERS_MAXIMIZE_CLEARANCE_VALID_STATE_SAMPLER_
#define OMPL_BASE_SAMPLERS_MAXIMIZE_CLEARANCE_VALID_STATE_SAMPLER_

#include "ompl/base/ValidStateSampler.h"
#include "ompl/base/StateSampler.h"

namespace ompl
{
    namespace base
    {
        /** \brief Generate valid samples randomly, but with a bias towards higher clearance. */
        class MaximizeClearanceValidStateSampler : public ValidStateSampler
        {
        public:
            /** \brief Constructor */
            MaximizeClearanceValidStateSampler(const SpaceInformation *si);

            ~MaximizeClearanceValidStateSampler() override;

            bool sample(State *state) override;

            bool sampleNear(State *state, const State *near, double distance) override;

            /** \brief The number of attempts at improving the clearance of the sampled state. */
            void setNrImproveAttempts(unsigned int attempts)
            {
                improveAttempts_ = attempts;
            }

            /** \brief Get the number of attempts to improve a sampled state */
            unsigned int getNrImproveAttempts() const
            {
                return improveAttempts_;
            }

        protected:
            /** \brief The sampler to build upon */
            StateSamplerPtr sampler_;

            /** \brief Number of attempts to improve a valid sample */
            unsigned int improveAttempts_;

        private:
            /** \brief Temporary work area */
            State *work_;
        };
    }
}

#endif
