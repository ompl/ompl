/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2021, Technische Universität Berlin (TU Berlin)
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
*   * Neither the name of the TU Berlin nor the names of its
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

/* Author: Francesco Grothe */

#ifndef OMPL_BASE_SPACES_ANIMATION_STATE_SPACE_
#define OMPL_BASE_SPACES_ANIMATION_STATE_SPACE_

#include "ompl/base/StateSpace.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>
#include <ompl/util/Exception.h>

namespace ompl
{
    namespace base
    {
        /** \brief A state space consisting of a space and a time component. */
        class SpaceTimeStateSpace : public CompoundStateSpace
        {
        public:
            /** \brief Constructor. The maximum velocity and the weight of the time component for distance calculation
                need to be specified. */
            explicit SpaceTimeStateSpace(const StateSpacePtr& spaceComponent, double vMax = 1.0, double timeWeight = 0.5);

            /** \brief The distance from state1 to state2. May be infinite.
             *
             * The distance is direction independent for compatibility to some planners (like RRTConnect).
             * Make sure that the motion validator does not allow motions backwards in time or adjust the distance
             * function in the Space Time State Space and the used planner.
             * */
            double distance(const ompl::base::State *state1, const ompl::base::State *state2) const override;

            /** \brief The time to get from state1 to state2 with respect to vMax. */
            double timeToCoverDistance(const ompl::base::State *state1, const ompl::base::State *state2) const;

            /** \brief The distance of just the space component. */
            double distanceSpace(const ompl::base::State *state1, const ompl::base::State *state2) const;

            /** \brief The distance of just the time component. */
            double distanceTime(const ompl::base::State *state1, const ompl::base::State *state2) const;

            /** \brief The time value of the given state. */
            static double getStateTime(const ompl::base::State *state) ;

            /** \brief Set lower and upper time bounds for the time component. */
            void setTimeBounds(double lb, double ub);

            /** \brief Maximum velocity getter. */
            double getVMax() const;

            /** \brief Maximum velocity setter. */
            void setVMax(double vMax);

            /** \brief The space component as a StateSpacePtr. */
            StateSpacePtr getSpaceComponent();

            /** \brief The time component as a TimeStateSpace pointer. */
            TimeStateSpace * getTimeComponent();

            /** \brief No metric state space, as the triangle inequality is not satisfied. */
            bool isMetricSpace() const override;

            /** \brief Maximum extent is infinite, as the distance can be infinite even with bounded time. */
            double getMaximumExtent() const override;

            /** \brief Scale epsilon appropriately after time or space bounds were set. */
            void updateEpsilon();

        protected:
            /** \brief The maximum velocity of the space. */
            double vMax_;

            /** \brief The epsilon for time distance calculation. */
            double eps_ = std::numeric_limits<float>::epsilon();
        };
    }
}

#endif
