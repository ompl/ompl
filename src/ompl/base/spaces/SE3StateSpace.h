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

#ifndef OMPL_BASE_SPACES_SE3_STATE_SPACE_
#define OMPL_BASE_SPACES_SE3_STATE_SPACE_

#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/SO3StateSpace.h"

namespace ompl
{
    namespace base
    {
        /** \brief A state space representing SE(3) */
        class SE3StateSpace : public CompoundStateSpace
        {
        public:
            /** \brief A state in SE(3): position = (x, y, z), quaternion = (x, y, z, w) */
            class StateType : public CompoundStateSpace::StateType
            {
            public:
                StateType() = default;

                /** \brief Get the X component of the state */
                double getX() const
                {
                    return as<RealVectorStateSpace::StateType>(0)->values[0];
                }

                /** \brief Get the Y component of the state */
                double getY() const
                {
                    return as<RealVectorStateSpace::StateType>(0)->values[1];
                }

                /** \brief Get the Z component of the state */
                double getZ() const
                {
                    return as<RealVectorStateSpace::StateType>(0)->values[2];
                }

                /** \brief Get the rotation component of the state */
                const SO3StateSpace::StateType &rotation() const
                {
                    return *as<SO3StateSpace::StateType>(1);
                }

                /** \brief Get the rotation component of the state and allow changing it as well */
                SO3StateSpace::StateType &rotation()
                {
                    return *as<SO3StateSpace::StateType>(1);
                }

                /** \brief Set the X component of the state */
                void setX(double x)
                {
                    as<RealVectorStateSpace::StateType>(0)->values[0] = x;
                }

                /** \brief Set the Y component of the state */
                void setY(double y)
                {
                    as<RealVectorStateSpace::StateType>(0)->values[1] = y;
                }

                /** \brief Set the Z component of the state */
                void setZ(double z)
                {
                    as<RealVectorStateSpace::StateType>(0)->values[2] = z;
                }

                /** \brief Set the X, Y and Z components of the state */
                void setXYZ(double x, double y, double z)
                {
                    setX(x);
                    setY(y);
                    setZ(z);
                }
            };

            SE3StateSpace()
            {
                setName("SE3" + getName());
                type_ = STATE_SPACE_SE3;
                addSubspace(std::make_shared<RealVectorStateSpace>(3), 1.0);
                addSubspace(std::make_shared<SO3StateSpace>(), 1.0);
                lock();
            }

            ~SE3StateSpace() override = default;

            /** \copydoc RealVectorStateSpace::setBounds() */
            void setBounds(const RealVectorBounds &bounds)
            {
                as<RealVectorStateSpace>(0)->setBounds(bounds);
            }

            /** \copydoc RealVectorStateSpace::getBounds() */
            const RealVectorBounds &getBounds() const
            {
                return as<RealVectorStateSpace>(0)->getBounds();
            }

            State *allocState() const override;
            void freeState(State *state) const override;

            void registerProjections() override;
        };
    }
}

#endif
