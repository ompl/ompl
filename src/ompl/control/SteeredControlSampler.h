/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Rice University
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
*   * Neither the name of Rice University nor the names of its
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

/* Author: Mark Moll, Ioan Sucan */

#ifndef OMPL_CONTROL_STEERED_CONTROL_SAMPLER_
#define OMPL_CONTROL_STEERED_CONTROL_SAMPLER_

#include "ompl/control/DirectedControlSampler.h"
#include "ompl/control/StatePropagator.h"
#include "ompl/control/SpaceInformation.h"
#include <cmath>

namespace ompl
{
    namespace control
    {
        /** \brief Abstract definition of a steered control sampler. It uses the
            steering function in a state propagator to find the controls that
            drive from one state to another. */
        class SteeredControlSampler : public DirectedControlSampler
        {
        public:
            /** \brief Constructor takes the state space to construct samples for as argument */
            SteeredControlSampler(const SpaceInformation *si) : DirectedControlSampler(si)
            {
            }

            ~SteeredControlSampler() override = default;

            unsigned int sampleTo(Control *control, const base::State *source, base::State *dest) override
            {
                double duration;
                if (!si_->getStatePropagator()->steer(source, dest, control, duration))
                    return 0;
                unsigned int steps = std::floor(duration / si_->getPropagationStepSize() + 0.5);
                return si_->propagateWhileValid(source, control, steps, dest);
            }

            unsigned int sampleTo(Control *control, const Control * /*previous*/, const base::State *source,
                                  base::State *dest) override
            {
                return sampleTo(control, source, dest);
            }
        };
    }
}

#endif
