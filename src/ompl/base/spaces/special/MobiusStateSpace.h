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

#ifndef OMPL_BASE_SPACES_MOBIUS_STATE_SPACE_
#define OMPL_BASE_SPACES_MOBIUS_STATE_SPACE_

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace ompl
{
    namespace base
    {
        /** \brief The Mobius strip is a 2-dimensional non-orientable surface.
         * */
        class MobiusStateSpace : public CompoundStateSpace
        {
        public:
            /** \brief The definition of a state (u,v) in the Mobius strip state
             * space.
             * The variable u is the position on the center circle in the interval [-Pi, Pi]
             * The variable v is the position along the width in the interval [-intervalMax, intervalMax]
             * */
            class StateType : public CompoundStateSpace::StateType
            {
            public:
                StateType() = default;

                double getU() const
                {
                    return as<SO2StateSpace::StateType>(0)->value;
                }
                double getV() const
                {
                    return as<RealVectorStateSpace::StateType>(1)->values[0];
                }

                void setU(double u)
                {
                    as<SO2StateSpace::StateType>(0)->value = u;
                }
                void setV(double v)
                {
                    as<RealVectorStateSpace::StateType>(1)->values[0] = v;
                }
                void setUV(double u, double v)
                {
                    setU(u);
                    setV(v);
                }
            };

            MobiusStateSpace(double intervalMax = 1.0, double radius = 1.0);

            virtual ~MobiusStateSpace() override = default;

            virtual double distance(const State *state1, const State *state2) const override;

            virtual void interpolate(const State *from, const State *to, double t, State *state) const override;

            virtual State *allocState() const override;

            /* \brief Convert a state to a 3D vector to visualize the state
             * space. */
            Eigen::Vector3f toVector(const State *state) const;

        private:
            double intervalMax_{1.0};  // width of mobius strip
            double radius_{1.0};       // radius of inner circle
        };
    }
}

#endif
