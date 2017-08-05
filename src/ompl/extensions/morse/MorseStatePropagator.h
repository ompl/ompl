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

/* Authors: Ioan Sucan, Caleb Voss */

#ifndef OMPL_EXTENSION_MORSE_STATE_PROPAGATOR_
#define OMPL_EXTENSION_MORSE_STATE_PROPAGATOR_

#include "ompl/control/SpaceInformation.h"
#include "ompl/extensions/morse/MorseEnvironment.h"

namespace ompl
{
    namespace control
    {
        /** \brief State propagation with MORSE. Only forward
            propagation is possible.

            At every propagation step, controls are applied using
            MorseEnvironment::applyControl(), and then \b worldStep()
            is called.*/
        class MorseStatePropagator : public StatePropagator
        {
        public:
            /** \brief Construct representation of a MORSE state propagator.
                If \e si->getStateSpace() does not cast to a
                MorseStateSpace, an exception is thrown. */
            MorseStatePropagator(const SpaceInformationPtr &si);

            virtual ~MorseStatePropagator()
            {
            }

            /** \brief Get the MORSE environment this state propagator operates on */
            const base::MorseEnvironmentPtr &getEnvironment() const
            {
                return env_;
            }

            /** \brief Will always return false, as the simulation can only proceed forward in time */
            bool canPropagateBackward() const override;

            /** \brief Propagate from a state, under a given control, for some specified amount of time */
            void propagate(const base::State *state, const Control *control, double duration,
                           base::State *result) const override;

        protected:
            /** \brief The MORSE environment this state propagator operates on */
            base::MorseEnvironmentPtr env_;
        };
    }
}

#endif
