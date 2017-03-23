/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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

/* Author: Dave Coleman <dave@dav.ee>
   Desc:   Find a valid sample with a minimum distance to nearby obstacles
           (clearance threshold)
*/

#ifndef OMPL_BASE_SAMPLERS_MINIMUM_CLEARANCE_VALID_STATE_SAMPLER_
#define OMPL_BASE_SAMPLERS_MINIMUM_CLEARANCE_VALID_STATE_SAMPLER_

#include "ompl/base/ValidStateSampler.h"
#include "ompl/base/StateSampler.h"

namespace ompl
{
    namespace base
    {
        /// @cond IGNORE
        OMPL_CLASS_FORWARD(MinimumClearanceValidStateSampler);
        /// @endcond

        /** \brief Generate valid samples randomly with extra requirement of min for clearance to nearest obstacle */
        class MinimumClearanceValidStateSampler : public ValidStateSampler
        {
        public:
            /** \brief Constructor */
            MinimumClearanceValidStateSampler(const SpaceInformation *si);

            ~MinimumClearanceValidStateSampler() override = default;
            ;

            bool sample(State *state) override;

            bool sampleNear(State *state, const State *near, double distance) override;

            /** \brief Set the minimum required distance of sample from nearest obstacle to be considered valid */
            void setMinimumObstacleClearance(double clearance)
            {
                clearance_ = clearance;
            }

            /** \brief Get the minimum required distance of sample from nearest obstacle to be considered valid */
            double getMinimumObstacleClearance() const
            {
                return clearance_;
            }

        protected:
            /** \brief The sampler to build upon */
            StateSamplerPtr sampler_;

            /** \brief Minimum required distance of sample from nearest obstacle to be considered valid */
            double clearance_;
        };
    }
}

#endif
