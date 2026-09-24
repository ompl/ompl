/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Rice University
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

/* Author: Clayton W. Ramsey */

#ifndef OMPL_BASE_OBJECTIVES_FLAT_EFFORT_OBJECTIVE_
#define OMPL_BASE_OBJECTIVES_FLAT_EFFORT_OBJECTIVE_

#include "ompl/base/OptimizationObjective.h"
#include "ompl/base/spaces/FlatStateSpace.h"

namespace ompl::base
{
    /// @cond IGNORE
    OMPL_CLASS_FORWARD(FlatEffortObjective);
    /// @endcond

    /** \brief An optimization objective for the steering polynomials on flat state spaces.

        One edge costs the control effort its polynomial spends plus the time penalty rho times its
        duration, so an optimizing planner trades effort against duration the same way the steering does
        when it picks a duration.
        Edge costs add along a path and states themselves cost nothing.

        Set this on any optimizing planner running in a flat state space.
        The stock ompl::base::PathLengthOptimizationObjective sums flat state distances instead, and a
        sum of distances isn't what a flat system spends getting anywhere.

        The space information has to carry an ompl::base::FlatStateSpace.
    */
    class FlatEffortObjective : public OptimizationObjective
    {
    public:
        /** \brief Construct a new objective function for the state space \e si. */
        FlatEffortObjective(const SpaceInformationPtr &si);

        /** \brief States cost nothing, since the whole cost of a path sits on its edges. */
        Cost stateCost(const State *s) const override;

        /** \brief Compute the cost of the steering polynomial from \e s1 to \e s2.

            Getting from a flat state to itself costs nothing, and a pair no polynomial reaches costs
            infinity.
        */
        Cost motionCost(const State *s1, const State *s2) const override;

        /** \brief Compute the cost of getting from \e s1 to \e s2 with nothing in the way.

            No route from \e s1 to \e s2 comes in under this, so it's admissible, and it's exact whenever
            the steering polynomial itself is clear.
        */
        Cost motionCostHeuristic(const State *s1, const State *s2) const override;

        /** \brief The cost of getting from \e s1 to \e s2 with nothing in the way. */
        Cost motionCostBestEstimate(const State *s1, const State *s2) const override;

    protected:
        /** \brief The flat state space the planning happens in. */
        const FlatStateSpace *stateSpace_;
    };
}  // namespace ompl::base

#endif
