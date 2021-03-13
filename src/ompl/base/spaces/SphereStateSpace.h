/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021,
 *  Max Planck Institute for Intelligent Systems (MPI-IS).
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
 *   * Neither the name of the MPI-IS nor the names
 *     of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written
 *     permission.
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

#ifndef OMPL_BASE_SPACES_SPHERE_STATE_SPACE_
#define OMPL_BASE_SPACES_SPHERE_STATE_SPACE_

#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/SO2StateSpace.h"

namespace ompl
{
    namespace base
    {
        class SphereStateSampler : public ompl::base::StateSampler
        {
        public:
            SphereStateSampler(const StateSpace *space);

            void sampleUniform(State *state) override;

            void sampleUniformNear(State *state, const State *near, double distance) override;

            void sampleGaussian(State *state, const State *mean, double stdDev) override;
        };

        class SphereStateSpace : public CompoundStateSpace
        {
        public:
            class StateType : public CompoundStateSpace::StateType
            {
            public:
                StateType() = default;

                double getTheta() const
                {
                    return as<SO2StateSpace::StateType>(0)->value;
                }
                double getPhi() const
                {
                    return as<SO2StateSpace::StateType>(1)->value;
                }

                void setTheta(double theta)
                {
                    as<SO2StateSpace::StateType>(0)->value = theta;
                }
                void setPhi(double phi)
                {
                    as<SO2StateSpace::StateType>(1)->value = phi;
                }
                void setThetaPhi(double theta, double phi)
                {
                    setTheta(theta);
                    setPhi(phi);
                }
            };

            SphereStateSpace();

            virtual ~SphereStateSpace() override = default;

            StateSamplerPtr allocDefaultStateSampler() const override;

            double getMeasure() const override;

            double distance(const State *state1, const State *state2) const override;

            State *allocState() const override;

        protected:
            double radius_{1.0};
        };
    }
}

#endif
