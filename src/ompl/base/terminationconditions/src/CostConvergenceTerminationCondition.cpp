/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, PickNik LLC
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
 *   * Neither the name of the PickNik LLC nor the names of its
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

/* Author: Henning Kayser, Mark Moll */

#include "ompl/base/terminationconditions/CostConvergenceTerminationCondition.h"

struct ompl::base::CostConvergenceTerminationCondition::Convergence
{
    Convergence(size_t window, double eps) : solutionsWindow(window), epsilon(eps)
    {
    }

    /// Adds a solution cost; returns true once the planner has converged.
    bool update(const Cost solutionCost)
    {
        ++solutions;
        size_t count = std::min(solutions, solutionsWindow);
        double newCost = ((count - 1) * averageCost + solutionCost.value()) / count;
        double costLowerThreshold = (1. - epsilon) * averageCost;
        double costUpperThreshold = (1. + epsilon) * averageCost;
        averageCost = newCost;

        if (count == solutionsWindow && averageCost > costLowerThreshold && averageCost < costUpperThreshold)
        {
            OMPL_DEBUG("CostConvergenceTerminationCondition: Cost of optimizing planner converged after %lu solutions",
                       solutions);
            return true;
        }
        return false;
    }

    /// Cumulative moving average of solutions found so far.
    double averageCost{0.};
    /// Number of solutions found so far.
    size_t solutions{0};
    /// Minimum number of solutions needed to decide whether the planner has converged.
    const size_t solutionsWindow;
    /// If an update to the cumulative moving average changes it by less than an epsilon fraction, convergence
    /// has been reached.
    const double epsilon;
};

ompl::base::CostConvergenceTerminationCondition::CostConvergenceTerminationCondition(
    ompl::base::ProblemDefinitionPtr &pdef, size_t solutionsWindow, double epsilon)
  : ompl::base::PlannerTerminationCondition(ompl::base::plannerNonTerminatingCondition())
  , pdef_(pdef)
  , convergence_(std::make_shared<Convergence>(solutionsWindow, epsilon))
{
    // The callback outlives this object on pdef_; a copy of the condition shares its terminate() flag.
    pdef_->setIntermediateSolutionCallback(
        [convergence = convergence_, ptc = static_cast<const PlannerTerminationCondition &>(*this)](
            const Planner * /*planner*/, const std::vector<const State *> & /*states*/, const Cost cost)
        {
            if (convergence->update(cost))
                ptc.terminate();
        });
}

void ompl::base::CostConvergenceTerminationCondition::processNewSolution(const ompl::base::Cost solutionCost)
{
    if (convergence_->update(solutionCost))
        terminate();
}
