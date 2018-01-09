/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017,
 *  National Institute of Advanced Industrial Science and
 *  Technology (AIST)
 *
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
 *   * Neither the name of the National Institute of Advanced Industrial
 *     Science and Technology nor the names of its contributors may be
 *     used to endorse or promote products derived from this software
 *     without specific prior written permission.
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

/* Author: Andreas Orthey */

#ifndef OMPL_BASE_SAMPLERS_BRIDGE_TEST_VALID_STATE_SAMPLER_
#define OMPL_BASE_SAMPLERS_BRIDGE_TEST_VALID_STATE_SAMPLER_

#include "ompl/base/StateSampler.h"
#include "ompl/base/ValidStateSampler.h"

namespace ompl
{
    namespace base
    {
        /** \brief Generate valid samples using bridge test. First
            sample an invalid state, then sample another invalid state. Take the midpoint of those samples.
            If midpoint is valid, return. If midpoint is invalid continue.

            @par External documentation
            Hsu, D., Jiang, T., Reif, J., & Sun, Z., The bridge test for sampling narrow
            passages with probabilistic roadmap planners. In <em>Robotics and
            Automation</em>, 2003.
            [[URL]](http://dx.doi.org/10.1109/ROBOT.2003.1242285)

        */

        class BridgeTestValidStateSampler : public ValidStateSampler
        {
        public:
            /** \brief Constructor */
            BridgeTestValidStateSampler(const SpaceInformation *si);

            ~BridgeTestValidStateSampler() override = default;

            bool sample(State *state) override;
            bool sampleNear(State *state, const State *near, double distance) override;

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
