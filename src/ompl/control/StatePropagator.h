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

#ifndef OMPL_CONTROL_STATE_PROPAGATOR_
#define OMPL_CONTROL_STATE_PROPAGATOR_

#include "ompl/control/Control.h"
#include "ompl/util/ClassForward.h"

namespace ompl
{

    namespace control
    {

        ClassForward(SpaceInformation);

        /** \brief Forward declaration of ompl::control::StatePropagator */
        ClassForward(StatePropagator);

        /** \class ompl::base::StatePropagatorPtr
            \brief A boost shared pointer wrapper for ompl::control::StatePropagator */

        class StatePropagator
        {
        public:

            /** \brief Constructor */
            StatePropagator(SpaceInformation* si) : si_(si)
            {
            }

            /** \brief Constructor */
            StatePropagator(const SpaceInformationPtr &si) : si_(si.get())
            {
            }

            virtual ~StatePropagator(void)
            {
            }

            /** \brief Propagate from a state, given a control, for some specified amount of time (the amount of time can
                also be negative, if canPropagateBackward() returns true)
                \param state the state to start propagating from
                \param control the control to apply
                \param duration the duration for which the control is applied
                \param result the state the system is brought to

                \note This function is <b>not used for integration</b>
                internally. If integrating a system of differential
                equations is needed, this should be implemented inside
                the propagate() function.

                \note The pointer to the starting state and the result
                state may be the same.
            */
            virtual void propagate(const base::State *state, const Control* control, const double duration, base::State *result) const = 0;

            /** \brief Some systems can only propagate forward in time (i.e., the \e duration argument for the propagate()
                function is always positive). If this is the case, this function should return false. Planners that need
                backward propagation (negative durations) will call this function to check. If backward propagation is
                possible, this function should return true (this is the default). */
            virtual bool canPropagateBackward(void) const
            {
                return true;
            }

        protected:

            /** \brief The instance of space information this state validity checker operates on */
            SpaceInformation *si_;

        };

    }
}

#endif
