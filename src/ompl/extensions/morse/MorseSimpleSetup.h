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

#ifndef OMPL_EXTENSION_MORSE_SIMPLE_SETUP_
#define OMPL_EXTENSION_MORSE_SIMPLE_SETUP_

#include "ompl/control/SimpleSetup.h"
#include "ompl/extensions/morse/MorseEnvironment.h"
#include "ompl/extensions/morse/MorseStateSpace.h"

namespace ompl
{
    namespace control
    {
        /** \brief Create the set of classes typically needed to solve a
            control problem when forward propagation is computed with MORSE */
        class MorseSimpleSetup : public SimpleSetup
        {
        public:
            /** \brief Pointer to the environment representing the MORSE simulation */
            const base::MorseEnvironmentPtr env_;

            /** \brief The control space is assumed to be
                MorseControlSpace. The state space is assumed to
                be MorseStateSpace. Constructor only needs the MORSE
                environment. */
            MorseSimpleSetup(const base::MorseEnvironmentPtr &env);

            virtual ~MorseSimpleSetup()
            {
            }

            /** \brief Get the MORSE environment associated with this setup */
            const base::MorseEnvironmentPtr &getEnvironment() const
            {
                return env_;
            }

            /** \brief Get the current MORSE state (read parameters from MORSE bodies) */
            base::ScopedState<base::MorseStateSpace> getCurrentState() const;

            /** \brief Set the current MORSE state (set parameters for MORSE bodies) */
            void setCurrentState(const base::ScopedState<> &state);

            /** \brief Set the current MORSE state (set parameters for MORSE bodies) */
            void setCurrentState(const base::State *state);

            /** \brief This method will create the necessary classes
                for planning. The solve() method will call this
                function automatically. */
            void setup() override;

            /** \brief Run the planner until solution is found or user shuts down MORSE */
            base::PlannerStatus solve();

            /** \brief Set the MORSE world to the states that are
                contained in a given path, sequentially. */
            void playPath(const base::PathPtr &path) const;

            /** \brief Call playPath() on the solution path, if one is available */
            void playSolutionPath() const;

            /** \brief Simulate the MORSE environment forward for \e steps simulation steps, using the control \e
               control.
                Construct a path representing this action. */
            base::PathPtr simulateControl(const double *control, unsigned int steps) const;

            /** \brief Simulate the MORSE environment forward for \e steps simulation steps, using the control \e
               control.
                Construct a path representing this action. */
            base::PathPtr simulateControl(const Control *control, unsigned int steps) const;

            /** \brief Simulate the MORSE environment forward for \e
                steps simulation steps, using the null control
                (ompl::control::ControlSpace::nullControl()).
                Construct a path representing this action. */
            base::PathPtr simulate(unsigned int steps) const;
        };
    }
}

#endif
