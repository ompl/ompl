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

#include "ompl/base/PlannerTerminationCondition.h"

namespace ompl
{
    namespace base
    {
        /// \brief: A termination condition for stopping an optimizing planner based on cost convergence
        class CostConvergenceTerminationCondition : public PlannerTerminationCondition
        {
        public:
            ///.\brief Constructor
            /// \param pdef Problem definition, needed to get access to the optimization
            /// objective and to set a callback to get intermediate solutions.
            /// \param solutionsWindow Minimum number of solutions to use in deciding
            /// whether a planner has converged.
            /// \param convergenceThreshold Threshold to consider for convergence. This should be a number close to 1.
            /// When lower cost is better (as in minimizing path length), convergenceThreshold should be less than 1.
            /// When higher cast is better (as in maximizing clearance), convergenceThreshold should be greater than 1.
            /// A number of 0.9 can be interpreted as: if the cost of the last found solution is greater than .9 times a
            /// moving average of the last found solutions, then the planner should terminate.
            CostConvergenceTerminationCondition(ProblemDefinitionPtr &pdef, size_t solutionsWindow = 10,
                                                double convergenceThreshold = 0.9);

            // Compute the running average cost of the last solutions (solutionWindow)
            // and terminate if the cost of a new solution is worse than
            // convergenceThreshold times the average cost.
            void processNewSolution(const Cost solutionCost);

        private:
            /// Shared pointer to problem definition.
            ProblemDefinitionPtr pdef_;
            /// Cumulative moving average of solutions found so far.
            Cost averageCost_{0.};
            /// Number of solutions found so far.
            size_t solutions_{0};

            /// Minimum number of solutions needed to decide whether the planner has converged.
            const size_t solutionsWindow_;
            /// Fraction of moving average that is used as a cost threshold for deciding convergence.
            const double convergenceThreshold_;
        };
    }  // namespace base
}  // namespace ompl
