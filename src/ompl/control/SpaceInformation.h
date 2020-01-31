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

#ifndef OMPL_CONTROL_SPACE_INFORMATION_
#define OMPL_CONTROL_SPACE_INFORMATION_

#include <utility>

#include "ompl/base/SpaceInformation.h"
#include "ompl/control/ControlSpace.h"
#include "ompl/control/ControlSampler.h"
#include "ompl/control/DirectedControlSampler.h"
#include "ompl/control/StatePropagator.h"
#include "ompl/control/Control.h"
#include "ompl/util/ClassForward.h"

namespace ompl
{
    /** \brief This namespace contains sampling based planning
        routines used by planning under differential constraints */
    namespace control
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::control::SpaceInformation */
        OMPL_CLASS_FORWARD(SpaceInformation);
        /// @endcond

        /** \class ompl::control::SpaceInformationPtr
            \brief A shared pointer wrapper for ompl::control::SpaceInformation */

        /** \brief A function that achieves state propagation.*/
        using StatePropagatorFn =
            std::function<void(const base::State *, const Control *, const double, base::State *)>;

        /** \brief Space information containing necessary information for planning with controls. setup() needs to be
         * called before use. */
        class SpaceInformation : public base::SpaceInformation
        {
        public:
            /** \brief Constructor. Sets the instance of the state and control spaces to plan with. */
            SpaceInformation(const base::StateSpacePtr &stateSpace, ControlSpacePtr controlSpace);

            ~SpaceInformation() override = default;

            /** \brief Get the control space */
            const ControlSpacePtr &getControlSpace() const
            {
                return controlSpace_;
            }

            /** @name Control memory management
                @{ */

            /** \brief Allocate memory for a control */
            Control *allocControl() const
            {
                return controlSpace_->allocControl();
            }

            /** \brief Free the memory of a control */
            void freeControl(Control *control) const
            {
                controlSpace_->freeControl(control);
            }

            /** \brief Copy a control to another */
            void copyControl(Control *destination, const Control *source) const
            {
                controlSpace_->copyControl(destination, source);
            }

            /** \brief Clone a control */
            Control *cloneControl(const Control *source) const
            {
                Control *copy = controlSpace_->allocControl();
                controlSpace_->copyControl(copy, source);
                return copy;
            }

            /** @} */

            /** @name Topology-specific control operations (as in the control space)
                @{ */

            /** \brief Print a control to a stream */
            void printControl(const Control *control, std::ostream &out = std::cout) const
            {
                controlSpace_->printControl(control, out);
            }

            /** \brief Check if two controls are the same */
            bool equalControls(const Control *control1, const Control *control2) const
            {
                return controlSpace_->equalControls(control1, control2);
            }

            /** \brief Make the control have no effect if it were to be applied to a state for any amount of time. */
            void nullControl(Control *control) const
            {
                controlSpace_->nullControl(control);
            }

            /** @} */

            /** @name Sampling of controls
                @{ */

            /** \brief Allocate a control sampler */
            ControlSamplerPtr allocControlSampler() const
            {
                return controlSpace_->allocControlSampler();
            }

            /** \brief Set the minimum and maximum number of steps a control is propagated for */
            void setMinMaxControlDuration(unsigned int minSteps, unsigned int maxSteps)
            {
                minSteps_ = minSteps;
                maxSteps_ = maxSteps;
            }

            /** \brief Set the minimum number of steps a control is propagated for */
            void setMinControlDuration(unsigned int minSteps)
            {
                minSteps_ = minSteps;
            }

            /** \brief Set the minimum and maximum number of steps a control is propagated for */
            void setMaxControlDuration(unsigned int maxSteps)
            {
                maxSteps_ = maxSteps;
            }

            /** \brief Get the minimum number of steps a control is propagated for */
            unsigned int getMinControlDuration() const
            {
                return minSteps_;
            }

            /** \brief Get the maximum number of steps a control is propagated for */
            unsigned int getMaxControlDuration() const
            {
                return maxSteps_;
            }

            /** \brief Allocate an instance of the DirectedControlSampler to use. This will be the default
               (SimpleDirectedControlSampler) unless
                setDirectedControlSamplerAllocator() was previously called. */
            DirectedControlSamplerPtr allocDirectedControlSampler() const;

            /** \brief Set the allocator to use for the  DirectedControlSampler */
            void setDirectedControlSamplerAllocator(const DirectedControlSamplerAllocator &dcsa);

            /** \brief Reset the DirectedControlSampler to be the default one */
            void clearDirectedSamplerAllocator();

            /** @} */

            /** @name Configuration of the state propagator
                @{ */

            /** \brief Get the instance of StatePropagator that performs state propagation */
            const StatePropagatorPtr &getStatePropagator() const
            {
                return statePropagator_;
            }

            /** \brief Set the function that performs state propagation */
            void setStatePropagator(const StatePropagatorFn &fn);

            /** \brief Set the instance of StatePropagator to perform state propagation */
            void setStatePropagator(const StatePropagatorPtr &sp);

            /** \brief When controls are applied to states, they are applied for a time duration that is an integer
                multiple of the stepSize, within the bounds specified by setMinMaxControlDuration() */
            void setPropagationStepSize(double stepSize)
            {
                stepSize_ = stepSize;
            }

            /** \brief Propagation is performed at integer multiples of a specified step size. This function returns the
             * value of this step size. */
            double getPropagationStepSize() const
            {
                return stepSize_;
            }
            /** @} */

            /** @name Primitives for propagating the model of the system
                @{ */

            /** \brief Propagate the model of the system forward, starting a a given state, with a given control, for a
               given number of steps.
                \param state the state to start at
                \param control the control to apply
                \param steps the number of time steps to apply the control for. Each time step is of length
               getPropagationStepSize()
                \param result the state at the end of the propagation */
            void propagate(const base::State *state, const Control *control, int steps, base::State *result) const;

            /** \brief Some systems can only propagate forward in time (i.e., the \e steps argument for the propagate()
                function is always positive). If this is the case, this function will return false. Planners that need
                backward propagation (negative \e steps) will call this function to check. If backward propagation is
                possible, this function will return true (this is the default). */
            bool canPropagateBackward() const;

            /** \brief Propagate the model of the system forward, starting at a given state, with a given control, for a
               given number of steps.
                Stop if a collision is found and return the number of steps actually performed without collision. If no
               collision is found, the returned value is
                equal to the \e steps argument. If a collision is found after the first step, the return value is 0 and
               \e result = \e state.
                \param state the state to start at
                \param control the control to apply
                \param steps the maximum number of time steps to apply the control for. Each time step is of length
               getPropagationStepSize(). If \e steps is negative, backward propagation will be performed.
                \param result the state at the end of the propagation or the last valid state if a collision is found */
            unsigned int propagateWhileValid(const base::State *state, const Control *control, int steps,
                                             base::State *result) const;

            /** \brief Propagate the model of the system forward, starting a a given state, with a given control, for a
               given number of steps.
                \param state the state to start at
                \param control the control to apply
                \param steps the number of time steps to apply the control for. Each time step is of length
               getPropagationStepSize(). If \e steps is negative, backward propagation will be performed.
                \param result the set of states along the propagated motion
                \param alloc flag indicating whether memory for the states in \e result should be allocated

                \note Start state \e state is not included in \e result */
            void propagate(const base::State *state, const Control *control, int steps,
                           std::vector<base::State *> &result, bool alloc) const;

            /** \brief Propagate the model of the system forward, starting at a given state, with a given control, for a
               given number of steps.
                Stop if a collision is found and return the number of steps actually performed without collision. If no
               collision is found, the returned value is
                equal to the \e steps argument.  If a collision is found after the first step, the return value is 0 and
               no states are added to \e result.
                If \e alloc is false and \e result cannot store all the generated states, propagation is stopped
               prematurely (when \e result is full).
                The starting state (\e state) is not included in \e result. The return value of the function indicates
               how many states have been written to \e result.

                \param state the state to start at
                \param control the control to apply
                \param steps the maximum number of time steps to apply the control for. Each time step is of length
               getPropagationStepSize(). If \e steps is negative, backward propagation will be performed.
                \param result the set of states along the propagated motion (only valid states included)
                \param alloc flag indicating whether memory for the states in \e result should be allocated
            */
            unsigned int propagateWhileValid(const base::State *state, const Control *control, int steps,
                                             std::vector<base::State *> &result, bool alloc) const;

            /** @} */

            /** \brief Print information about the current instance of the state space */
            void printSettings(std::ostream &out = std::cout) const override;

            /** \brief Perform additional setup tasks (run once, before use) */
            void setup() override;

        protected:
            /** Declare parameter settings */
            void declareParams();

            /** \brief The control space describing the space of controls applicable to states in the state space */
            ControlSpacePtr controlSpace_;

            /** \brief The state propagator used to model the motion of the system being planned for */
            StatePropagatorPtr statePropagator_;

            /** \brief The minimum number of steps to apply a control for */
            unsigned int minSteps_{0};

            /** \brief The maximum number of steps to apply a control for */
            unsigned int maxSteps_{0};

            /** \brief Optional allocator for the DirectedControlSampler. If not specified, the default implementation
             * is used */
            DirectedControlSamplerAllocator dcsa_;

            /** \brief The actual duration of each step */
            double stepSize_{0.};
        };
    }
}

#endif
