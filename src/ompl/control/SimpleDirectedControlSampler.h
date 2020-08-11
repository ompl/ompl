/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

#ifndef OMPL_CONTROL_SIMPLE_DIRECTED_CONTROL_SAMPLER_
#define OMPL_CONTROL_SIMPLE_DIRECTED_CONTROL_SAMPLER_

#include "ompl/control/DirectedControlSampler.h"
#include "ompl/control/ControlSampler.h"

namespace ompl
{
    namespace control
    {
        /** \brief Implementation of a simple directed control
            sampler. This is a basic implementation that does not
            actually take direction into account and builds upon ControlSampler.
            Instead, a set of k random controls are sampled, and the control that
            gets the system closest to the target state is returned.

           @par External documentation
           K-control sampling is first believed to be proposed in:

           S.M. LaValle and J.J. Kuffner, Randomized kinodynamic planning, <em>Intl. J. of Robotics Research</em>, vol.
           20, pp. 378–400, May 2001. DOI: [10.1177/02783640122067453](http://dx.doi.org/10.1177/02783640122067453)<br>
           [[PDF]](http://ijr.sagepub.com/content/20/5/378.full.pdf)
           [[more]](http://msl.cs.uiuc.edu/~lavalle/rrtpubs.html) */
        class SimpleDirectedControlSampler : public DirectedControlSampler
        {
        public:
            /** \brief Constructor takes the state space to construct samples for as argument
                Optionally, a \e k value can be given to indicate the number of controls to
                try when directing a system toward a specific state.  Default value is 1. */
            SimpleDirectedControlSampler(const SpaceInformation *si, unsigned int k = 1);

            ~SimpleDirectedControlSampler() override;

            /** \brief Retrieve the number of controls to generate when finding the best control. */
            unsigned int getNumControlSamples() const
            {
                return numControlSamples_;
            }

            /** \brief Set the number of controls to generate when finding the best control. */
            void setNumControlSamples(unsigned int numSamples)
            {
                numControlSamples_ = numSamples;
            }

            /** \brief Sample a control given that it will be applied
                to state \e state and the intention is to reach state
                \e dest. This is useful for some algorithms that
                have a notion of direction in their exploration (e.g.,
                \ref cRRT). Furthermore, return the duration for which
                this control should be applied. The state
                \e dest is modified to match the state reached with the computed
                control and duration. The motion is checked for validity. */
            unsigned int sampleTo(Control *control, const base::State *source, base::State *dest) override;

            /** \brief Sample a control given that it will be applied
                to state \e state and the intention is to reach state
                \e dest. Also take into account the fact that the
                previously applied control is \e previous. This is
                useful for some algorithms that have a notion of
                direction in their exploration (e.g.,
                \ref cRRT). Furthermore, return the duration for which
                this control should be applied. The state \e dest is
                modified to match the state reached with the computed
                control and duration. The motion is checked for validity. */
            unsigned int sampleTo(Control *control, const Control *previous, const base::State *source,
                                  base::State *dest) override;

        protected:
            /** \brief Samples \e numControlSamples_ controls, and returns the
                control that brings the system the closest to \e target */
            virtual unsigned int getBestControl(Control *control, const base::State *source, base::State *dest,
                                                const Control *previous);

            /** \brief An instance of the control sampler*/
            ControlSamplerPtr cs_;

            /** \brief The number of controls to sample when finding the best control*/
            unsigned int numControlSamples_;
        };
    }
}

#endif
