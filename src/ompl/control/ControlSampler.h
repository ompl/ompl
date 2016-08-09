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

#ifndef OMPL_CONTROL_CONTROL_SAMPLER_
#define OMPL_CONTROL_CONTROL_SAMPLER_

#include "ompl/base/State.h"
#include "ompl/control/Control.h"
#include "ompl/util/RandomNumbers.h"
#include "ompl/util/ClassForward.h"
#include <vector>
#include <functional>

namespace ompl
{
    namespace control
    {
        /// @cond IGNORE
        OMPL_CLASS_FORWARD(ControlSpace);
        /// @endcond

        /// @cond IGNORE
        /** \brief Forward declaration of ompl::control::ControlSampler */
        OMPL_CLASS_FORWARD(ControlSampler);
        /// @endcond

        /** \class ompl::control::ControlSamplerPtr
            \brief A shared pointer wrapper for ompl::control::ControlSampler */

        /** \brief Abstract definition of a control sampler. Motion
            planners that need to sample controls will call functions
            from this class. Planners should call the versions of
            sample() and sampleNext() with most arguments, whenever
            this information is available. */
        class ControlSampler
        {
        public:
            // non-copyable
            ControlSampler(const ControlSampler &) = delete;
            ControlSampler &operator=(const ControlSampler &) = delete;

            /** \brief Constructor takes the state space to construct samples for as argument */
            ControlSampler(const ControlSpace *space) : space_(space)
            {
            }

            virtual ~ControlSampler() = default;

            /** \brief Sample a control. All other control sampling
                functions default to this one, unless a user-specified
                implementation is given. */
            virtual void sample(Control *control) = 0;

            /** \brief Sample a control, given it is applied to a
                specific state (\e state). The default implementation calls the
                previous definition of sample(). Providing a different
                implementation of this function is useful if, for
                example, the sampling of controls depends on the state
                of the system. When attempting to sample controls that
                keep a system stable, for example, knowing the state
                at which the control is applied is important. */
            virtual void sample(Control *control, const base::State *state);

            /** \brief Sample a control, given the previously applied
                control. The default implementation calls the first
                definition of sample(). For some systems it is
                possible that large changes in controls are not
                desirable. For example, switching from maximum
                acceleration to maximum deceleration is not desirable
                when driving a car. */
            virtual void sampleNext(Control *control, const Control *previous);

            /** \brief Sample a control, given the previously applied
                control and that it is applied to a specific
                state. The default implementation calls the first
                definition of sample(), even if other implementations
                of the sampleNext() shown above are provided. Often
                this function needs to be overridden as it is the
                function planners typically call.  */
            virtual void sampleNext(Control *control, const Control *previous, const base::State *state);

            /** \brief Sample a number of steps to execute a control for */
            virtual unsigned int sampleStepCount(unsigned int minSteps, unsigned int maxSteps);

        protected:
            /** \brief The control space this sampler operates on */
            const ControlSpace *space_;

            /** \brief Instance of random number generator */
            RNG rng_;
        };

        /** \brief Definition of a compound control sampler. This is useful to construct samplers for compound controls.
         */
        class CompoundControlSampler : public ControlSampler
        {
        public:
            /** \brief Constructor */
            CompoundControlSampler(const ControlSpace *space) : ControlSampler(space)
            {
            }

            /** \brief Destructor. This frees the added samplers as well. */
            ~CompoundControlSampler() override = default;

            /** \brief Add a sampler as part of the new compound
                sampler. This sampler is used to sample part of the
                compound control.  */
            virtual void addSampler(const ControlSamplerPtr &sampler);

            void sample(Control *control) override;
            void sample(Control *control, const base::State *state) override;
            void sampleNext(Control *control, const Control *previous) override;
            void sampleNext(Control *control, const Control *previous, const base::State *state) override;

        protected:
            /** \brief The instances of samplers used for compound sampler */
            std::vector<ControlSamplerPtr> samplers_;

        private:
            /** \brief Number of sampler instances */
            unsigned int samplerCount_;
        };

        /** \brief Definition of a function that can allocate a control sampler */
        using ControlSamplerAllocator = std::function<ControlSamplerPtr(const ControlSpace *)>;
    }
}

#endif
