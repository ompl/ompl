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

#ifndef OMPL_BASE_MANIFOLDS_SE2_STATE_MANIFOLD_
#define OMPL_BASE_MANIFOLDS_SE2_STATE_MANIFOLD_

#include "ompl/base/StateManifold.h"
#include "ompl/base/manifolds/RealVectorStateManifold.h"
#include "ompl/base/manifolds/SO2StateManifold.h"

namespace ompl
{
    namespace base
    {

        /** \brief A manifold representing SE(2) */
        class SE2StateManifold : public CompoundStateManifold
        {
        public:

            /** \brief A state in SE(2): (x, y, yaw) */
            class StateType : public CompoundStateManifold::StateType
            {
            public:
                StateType(void) : CompoundStateManifold::StateType()
                {
                }

                /** \brief Get the X component of the state */
                double getX(void) const
                {
                    return as<RealVectorStateManifold::StateType>(0)->values[0];
                }

                /** \brief Get the Y component of the state */
                double getY(void) const
                {
                    return as<RealVectorStateManifold::StateType>(0)->values[1];
                }

                /** \brief Get the yaw component of the state. This is
                    the rotation in plane, with respect to the Z
                    axis. */
                double getYaw(void) const
                {
                    return as<SO2StateManifold::StateType>(1)->value;
                }

                /** \brief Set the X component of the state */
                void setX(double x)
                {
                    as<RealVectorStateManifold::StateType>(0)->values[0] = x;
                }

                /** \brief Set the Y component of the state */
                void setY(double y)
                {
                    as<RealVectorStateManifold::StateType>(0)->values[1] = y;
                }

                /** \brief Set the X and Y components of the state */
                void setXY(double x, double y)
                {
                    setX(x);
                    setY(y);
                }

                /** \brief Set the yaw component of the state. This is
                    the rotation in plane, with respect to the Z
                    axis. */
                void setYaw(double yaw)
                {
                    as<SO2StateManifold::StateType>(1)->value = yaw;
                }

            };


            SE2StateManifold(void) : CompoundStateManifold()
            {
                name_ = "SE2" + name_;
                type_ = STATE_MANIFOLD_SE2;
                addSubManifold(StateManifoldPtr(new RealVectorStateManifold(2)), 1.0);
                addSubManifold(StateManifoldPtr(new SO2StateManifold()), 0.5);
                lock();
            }

            virtual ~SE2StateManifold(void)
            {
            }

            /** \copydoc RealVectorStateManifold::setBounds() */
            void setBounds(const RealVectorBounds &bounds)
            {
                as<RealVectorStateManifold>(0)->setBounds(bounds);
            }

            /** \copydoc RealVectorStateManifold::getBounds() */
            const RealVectorBounds& getBounds(void) const
            {
                return as<RealVectorStateManifold>(0)->getBounds();
            }

            virtual State* allocState(void) const;
            virtual void freeState(State *state) const;

            virtual void registerProjections(void);

        };
    }
}

#endif
