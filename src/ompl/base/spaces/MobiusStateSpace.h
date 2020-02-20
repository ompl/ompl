/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2020, Stuttgart University
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
*   * Neither the name of the Stuttgart University nor the names of its
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

/* Author: Andreas Orthey */

#ifndef OMPL_BASE_SPACES_MOBIUS_STATE_SPACE_
#define OMPL_BASE_SPACES_MOBIUS_STATE_SPACE_

#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/SO2StateSpace.h"

namespace ompl
{
    namespace base
    {
        class MobiusStateSpace : public CompoundStateSpace
        {
        public:
            class StateType : public CompoundStateSpace::StateType
            {
            public:
                StateType() = default;

                double getX() const
                {
                    return as<RealVectorStateSpace::StateType>(0)->values[0];
                }

                double getS1() const
                {
                    return as<SO2StateSpace::StateType>(1)->value;
                }

                void setX(double x)
                {
                    as<RealVectorStateSpace::StateType>(0)->values[0] = x;
                }

                void setS1(double angle)
                {
                    as<SO2StateSpace::StateType>(1)->value = angle;
                }
            };

            MobiusStateSpace()
            {
                setName("Mobius" + getName());
                type_ = STATE_SPACE_MOBIUS;
                addSubspace(std::make_shared<RealVectorStateSpace>(2), 1.0);
                addSubspace(std::make_shared<SO2StateSpace>(), 0.5);
                lock();
            }

            ~MobiusStateSpace() override = default;

            void setBounds(const RealVectorBounds &bounds)
            {
                as<RealVectorStateSpace>(0)->setBounds(bounds);
            }

            const RealVectorBounds &getBounds() const
            {
                return as<RealVectorStateSpace>(0)->getBounds();
            }

            State *allocState() const override;
            void freeState(State *state) const override;

            virtual void interpolate(
                const State *from, 
                const State *to, 
                double t, 
                State *state) const override;

            // virtual double distance(
            //     const State *state1, 
            //     const State *state2) const override;


            // void registerProjections() override;
        };
    }
}

#endif
