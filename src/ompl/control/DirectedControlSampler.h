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

#ifndef OMPL_CONTROL_DIRECTED_CONTROL_SAMPLER_
#define OMPL_CONTROL_DIRECTED_CONTROL_SAMPLER_

#include "ompl/base/State.h"
#include "ompl/control/Control.h"
#include "ompl/util/ClassForward.h"
#include <functional>

namespace ompl
{
    namespace control
    {
        /// @cond IGNORE
        OMPL_CLASS_FORWARD(SpaceInformation);
        OMPL_CLASS_FORWARD(DirectedControlSampler);
        /// @endcond

        /** \class ompl::control::DirectedControlSamplerPtr
            \brief A shared pointer wrapper for ompl::control::DirectedControlSampler */

        /** \brief Abstract definition of a directed control sampler. Motion
            planners that need to sample controls that take the system to a desired direction will call functions
            from this class. Planners should call the versions of sampleTo() with most arguments, whenever this
           information is available.
            If no direction information is available, the use of a ControlSampler is perhaps more appropriate. */
        class DirectedControlSampler
        {
        public:
            // non-copyable
            DirectedControlSampler(const DirectedControlSampler &) = delete;
            DirectedControlSampler &operator=(const DirectedControlSampler &) = delete;

            /** \brief Constructor takes the state space to construct samples for as argument */
            DirectedControlSampler(const SpaceInformation *si) : si_(si)
            {
            }

            virtual ~DirectedControlSampler() = default;

            /** \brief Sample a control given that it will be applied to state
                \e state and the intention is to reach state \e target. This is
                useful for some algorithms that have a notion of direction in
                their exploration (e.g., \ref cRRT). Furthermore, return the
                duration for which this control should be applied. The state
                \e dest is modified to match the state reached with the computed
                control and duration. The motion is checked for validity. */
            virtual unsigned int sampleTo(Control *control, const base::State *source, base::State *dest) = 0;

            /** \brief Sample a control given that it will be applied to state
                \e state and the intention is to reach state \e dest. Also take
                into account the fact that the previously applied control is \e
                previous. This is useful for some algorithms that have a notion
                of direction in their exploration (e.g., \ref cRRT).
                Furthermore, return the duration for which this control should
                be applied. The state \e dest is modified to match the state
                reached with the computed control and duration. The motion is
                checked for validity. */
            virtual unsigned int sampleTo(Control *control, const Control *previous, const base::State *source,
                                          base::State *dest) = 0;

        protected:
            /** \brief The space information this sampler operates on */
            const SpaceInformation *si_;
        };

        /** \brief Definition of a function that can allocate a directed control sampler */
        using DirectedControlSamplerAllocator = std::function<DirectedControlSamplerPtr(const SpaceInformation *)>;
    }
}

#endif
