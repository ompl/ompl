/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
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

#ifndef OMPL_BASE_SAMPLERS_GAUSSIAN_VALID_STATE_SAMPLER_
#define OMPL_BASE_SAMPLERS_GAUSSIAN_VALID_STATE_SAMPLER_

#include "ompl/base/ValidStateSampler.h"
#include "ompl/base/StateSampler.h"

namespace ompl
{
    namespace base
    {
        /** \brief Generate valid samples using the Gaussian sampling strategy */
        class GaussianValidStateSampler : public ValidStateSampler
        {
        public:
            /** \brief Constructor */
            GaussianValidStateSampler(const SpaceInformation *si);

            virtual ~GaussianValidStateSampler()
            {
            }

            virtual bool sample(State *state);
            virtual bool sampleNear(State *state, const State *near, const double distance);

            /** \brief Get the standard deviation used when sampling */
            double getStdDev() const
            {
                return stddev_;
            }

            /** \brief Set the standard deviation to use when sampling */
            void setStdDev(double stddev)
            {
                stddev_ = stddev;
            }

        protected:
            /** \brief The sampler to build upon */
            StateSamplerPtr sampler_;

            /** \brief The standard deviation to use in the sampling process */
            double stddev_;
        };
    }
}

#endif
