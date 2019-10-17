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

ompl::base::CostConvergenceTerminationCondition::CostConvergenceTerminationCondition(
    ompl::base::ProblemDefinitionPtr &pdef, size_t solutionsWindow, double epsilon)
    : ompl::base::PlannerTerminationCondition(ompl::base::plannerNonTerminatingCondition()),
      pdef_(pdef),
      solutionsWindow_(solutionsWindow),
      epsilon_(epsilon)
{
    pdef_->setIntermediateSolutionCallback(
	    [this](const Planner* /*planner*/, const std::vector<const State*>& /*states*/, const Cost cost) {
	        this->processNewSolution(cost);
	    });
}

void ompl::base::CostConvergenceTerminationCondition::processNewSolution(const ompl::base::Cost solutionCost)
{
    ++solutions_;
    size_t solutions = std::min(solutions_, solutionsWindow_);
    double newCost = ((solutions - 1) * averageCost_ + solutionCost.value()) / solutions;
    double costLowerThreshold = (1. - epsilon_) * averageCost_;
    double costUpperThreshold = (1. + epsilon_) * averageCost_;
    averageCost_ = newCost;

    if (solutions == solutionsWindow_ &&
        averageCost_ > costLowerThreshold &&
        averageCost_ < costUpperThreshold)
    {
        OMPL_DEBUG("CostConvergenceTerminationCondition: Cost of optimizing planner converged after %lu solutions", solutions_);
        terminate();
    }
}
