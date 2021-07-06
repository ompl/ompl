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

#ifndef OMPL_BASE_SPACES_KLEINBOTTLE_STATE_SPACE_
#define OMPL_BASE_SPACES_KLEINBOTTLE_STATE_SPACE_

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace ompl
{
    namespace base
    {
        /** \brief State sampler for the Klein bottle state space */
        class KleinBottleStateSampler : public StateSampler
        {
        public:
            KleinBottleStateSampler(const StateSpace *space);

            void sampleUniform(State *state) override;

            void sampleUniformNear(State *state, const State *near, double distance) override;

            void sampleGaussian(State *state, const State *mean, double stdDev) override;

        private:
            /** \brief Constant of maximum norm gradient of surface (do not change).
             * Estimated by taking the upper bound over 10k samples. */
            const double gMax_{4.1455};
        };

        /** \brief The Klein bottle is a 2-dimensional non-orientable surface.
         * In this class, we implement a 3-dimensional immersion of the
         * Bottle.  */
        class KleinBottleStateSpace : public CompoundStateSpace
        {
        public:
            /** \brief The definition of a state (u,v) in the Klein bottle state
             * space. A state is represented as a cylinder with height u in the
             * interval [0, Pi] and
             * angle v in the interval [-Pi, Pi] as in the discussion
             * here: https://en.wikipedia.org/wiki/Klein_bottle#Construction
             *
             * This cylinder is then glued together (mapped) internally to provide
             * correct distance and interpolation functions. */
            class StateType : public CompoundStateSpace::StateType
            {
            public:
                StateType() = default;

                /** \brief Access U, the height of the cylinder */
                double getU() const
                {
                    return as<RealVectorStateSpace::StateType>(0)->values[0];
                }
                /** \brief Access V, the angle of the cylinder */
                double getV() const
                {
                    return as<SO2StateSpace::StateType>(1)->value;
                }

                /** \brief Set U, the height of the cylinder */
                void setU(double u)
                {
                    as<RealVectorStateSpace::StateType>(0)->values[0] = u;
                }
                /** \brief Set V, the angle of the cylinder */
                void setV(double v)
                {
                    as<SO2StateSpace::StateType>(1)->value = v;
                }
                void setUV(double u, double v)
                {
                    setU(u);
                    setV(v);
                }
            };

            KleinBottleStateSpace();

            ~KleinBottleStateSpace() override = default;

            StateSamplerPtr allocDefaultStateSampler() const override;

            double distance(const State *state1, const State *state2) const override;

            void interpolate(const State *from, const State *to, double t, State *state) const override;

            State *allocState() const override;

            /* \brief Convert a state to a 3D vector to visualize the state
             * space. */
            Eigen::Vector3f toVector(const State *state) const;
        };
    }
}

#endif
