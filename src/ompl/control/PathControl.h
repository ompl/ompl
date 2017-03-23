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

#ifndef OMPL_CONTROL_PATH_CONTROL_
#define OMPL_CONTROL_PATH_CONTROL_

#include "ompl/control/SpaceInformation.h"
#include "ompl/base/Path.h"
#include "ompl/geometric/PathGeometric.h"
#include <vector>

namespace ompl
{
    namespace base
    {
        /// @cond IGNORE
        OMPL_CLASS_FORWARD(OptimizationObjective);
        /// @endcond
    }

    namespace control
    {
        /** \brief Definition of a control path.

         This is the type of path produced when planning with
         differential constraints. */
        class PathControl : public base::Path
        {
        public:
            /** \brief Constructor */
            PathControl(const base::SpaceInformationPtr &si);

            /** \brief Copy constructor */
            PathControl(const PathControl &path);

            ~PathControl() override
            {
                freeMemory();
            }

            /** \brief Assignment operator */
            PathControl &operator=(const PathControl &other);

            /** \brief Not yet implemented. */
            base::Cost cost(const base::OptimizationObjectivePtr &opt) const override;

            /** \brief The path length (sum of control durations) */
            double length() const override;

            /** \brief Check if the path is valid */
            bool check() const override;

            /** \brief Print the path to a stream */
            void print(std::ostream &out) const override;
            /** \brief Print the path as a real-valued matrix where the
                i-th row represents the i-th state along the path, followed
                by the control and duration needed to reach this state. For
                the first state the control and duration are zeroes. The
                state components printed are those returned by
                ompl::base::StateSpace::copyToReals, while the control
                components printed are the discrete components (if any)
                followed by the real-valued ones as returned by
                ompl::control::ControlSpace::getValueAddressAtIndex. */
            virtual void printAsMatrix(std::ostream &out) const;

            /** \brief Convert this path into a geometric path (interpolation is performed and then states are copied)
             */
            geometric::PathGeometric asGeometric() const;

            /** @name Path operations
                @{ */

            /** \brief Append \e state to the end of the path; it is assumed \e state is the first state, so no control
               is applied.
                The memory for \e state is copied. There are no checks to make sure the number of controls and states
               make sense. */
            void append(const base::State *state);

            /** \brief Append \e state to the end of the path and assume \e control is applied for the duration \e
               duration.
                The memory for \e state and for \e control is copied. There are no checks to make sure the number of
               controls and states make sense. */
            void append(const base::State *state, const Control *control, double duration);

            /** \brief Make the path such that all controls are applied for a single time step (computes intermediate
             * states) */
            void interpolate();

            /** \brief Set this path to a random segment */
            void random();

            /** \brief Set this path to a random valid segment. Sample \e attempts times for valid segments. Returns
             * true on success.*/
            bool randomValid(unsigned int attempts);

            /** @} */

            /** @name Functionality for accessing states and controls
                @{ */

            /** \brief Get the states that make up the path (as a reference, so it can be modified, hence the function
             * is not const) */
            std::vector<base::State *> &getStates()
            {
                return states_;
            }

            /** \brief Get the controls that make up the path (as a reference, so it can be modified, hence the function
             * is not const) */
            std::vector<Control *> &getControls()
            {
                return controls_;
            }

            /** \brief Get the control durations used along the path (as a reference, so it can be modified, hence the
             * function is not const) */
            std::vector<double> &getControlDurations()
            {
                return controlDurations_;
            }

            /** \brief Get the state located at \e index along the path */
            base::State *getState(unsigned int index)
            {
                return states_[index];
            }

            /** \brief Get the state located at \e index along the path */
            const base::State *getState(unsigned int index) const
            {
                return states_[index];
            }

            /** \brief Get the control located at \e index along the path. This is the control that gets applied to the
             * state located at \e index */
            Control *getControl(unsigned int index)
            {
                return controls_[index];
            }

            /** \brief Get the control located at \e index along the path. This is the control that gets applied to the
             * state located at \e index */
            const Control *getControl(unsigned int index) const
            {
                return controls_[index];
            }

            /** \brief Get the duration of the control at \e index, which gets applied to the state at \e index. */
            double getControlDuration(unsigned int index) const
            {
                return controlDurations_[index];
            }

            /** \brief Get the number of states (way-points) that make up this path */
            std::size_t getStateCount() const
            {
                return states_.size();
            }

            /** \brief Get the number of controls applied along this path. This should be equal to getStateCount() - 1
             * unless there are 0 states, in which case the number of controls will also be 0. */
            std::size_t getControlCount() const
            {
                return controls_.size();
            }

            /** @} */

        protected:
            /** \brief The list of states that make up the path */
            std::vector<base::State *> states_;

            /** \brief The control applied at each state. This array contains one element less than the list of states
             */
            std::vector<Control *> controls_;

            /** \brief The duration of the control applied at each state. This array contains one element less than the
             * list of states */
            std::vector<double> controlDurations_;

            /** \brief Free the memory allocated by the path */
            void freeMemory();

            /** \brief Copy the content of a path to this one */
            void copyFrom(const PathControl &other);
        };
    }
}

#endif
