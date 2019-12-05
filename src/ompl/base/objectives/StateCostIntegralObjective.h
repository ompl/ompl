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

/* Author: Luis G. Torres */

#ifndef OMPL_BASE_OBJECTIVES_STATE_COST_OPTIMIZATION_OBJECTIVE_
#define OMPL_BASE_OBJECTIVES_STATE_COST_OPTIMIZATION_OBJECTIVE_

#include "ompl/base/OptimizationObjective.h"

namespace ompl
{
    namespace base
    {
        /** \brief Defines optimization objectives where path cost can
            be represented as a path integral over a cost function
            defined over the state space. This cost function is
            specified by implementing the stateCost() method. */
        class StateCostIntegralObjective : public OptimizationObjective
        {
        public:
            /** \brief If enableMotionCostInterpolation is set to
                true, then calls to motionCost() will divide the
                motion segment into smaller parts (the number of parts
                being defined by StateSpace::validSegmentCount()) for
                more accurate cost integral computation (but this
                takes more computation time). If
                enableMotionCostInterpolation is false (the default),
                only the two endpoint states are used for motion cost
                computation.
            */
            StateCostIntegralObjective(const SpaceInformationPtr &si, bool enableMotionCostInterpolation = false);

            /** \brief Returns a cost with a value of 1. */
            Cost stateCost(const State *s) const override;

            /** \brief Compute the cost of a path segment from \e s1 to \e s2 (including endpoints)
                \param s1 start state of the motion to be evaluated
                \param s2 final state of the motion to be evaluated
                \param cost the cost of the motion segment

                By default, this function computes
                \f{eqnarray*}{
                \mbox{cost} &=& \frac{cost(s_1) + cost(s_2)}{2}\vert s_1 - s_2 \vert
                \f}

                If enableMotionCostInterpolation was specified as true
                in constructing this object, the cost will be computed
                by separating the motion into
                StateSpace::validSegmentCount() segments, using the
                above formula to compute the cost of each of those
                segments, and adding them up.
            */
            Cost motionCost(const State *s1, const State *s2) const override;

            /** \brief Estimate the cost of a path segment from \e s1 to \e s2 (including endpoints).
                \param s1 start state of the motion to be evaluated
                \param s2 final state of the motion to be evaluated
                \param cost the cost of the motion segment

                This function computes
                \f{eqnarray*}{
                \mbox{cost} &=& \frac{cost(s_1) + cost(s_2)}{2}\vert s_1 - s_2 \vert
                \f}
                regardless of whether enableMotionCostInterpolation was specified as true in
                constructing this object.
            */
            Cost motionCostBestEstimate(const State *s1, const State *s2) const override;

            /** \brief Returns whether this objective subdivides
                motions into smaller segments for more accurate motion
                cost computation. Motion cost interpolation is
                disabled by default.
            */
            bool isMotionCostInterpolationEnabled() const;

        protected:
            /** \brief If true, then motionCost() will more accurately compute
                the cost of a motion by taking small steps along the
                motion and accumulating the cost. This sacrifices speed
                for accuracy. If false, the motion cost will be
                approximated by taking the average of the costs at the
                two end points, and normalizing by the distance between
                the two end points. */
            bool interpolateMotionCost_;

            /** \brief Helper method which uses the trapezoidal rule
                to approximate the integral of the cost between two
                states of distance \e dist and costs \e c1 and \e
                c2 */
            Cost trapezoid(Cost c1, Cost c2, double dist) const
            {
                return Cost(0.5 * dist * (c1.value() + c2.value()));
            }
        };
    }
}

#endif
