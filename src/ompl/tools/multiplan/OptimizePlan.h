/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
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

#ifndef OMPL_TOOLS_MULTIPLAN_OPTIMIZE_PLAN_
#define OMPL_TOOLS_MULTIPLAN_OPTIMIZE_PLAN_

#include "ompl/tools/multiplan/ParallelPlan.h"

namespace ompl
{
    namespace tools
    {
        /** \brief Run one or more motion planners repeatedly (using a
            specified number of threads), and hybridize solutions, trying
            to optimize solutions. */
        class OptimizePlan
        {
        public:
            /** \brief Create an instance for a specified space information */
            OptimizePlan(const base::ProblemDefinitionPtr &pdef) : pp_(pdef)
            {
            }

            virtual ~OptimizePlan() = default;

            /** \brief Add a planner to use. */
            void addPlanner(const base::PlannerPtr &planner);

            /** \brief Add a planner allocator to use. */
            void addPlannerAllocator(const base::PlannerAllocator &pa);

            /** \brief Clear the set of planners to be executed */
            void clearPlanners();

            /** \brief Get the problem definition used */
            const base::ProblemDefinitionPtr &getProblemDefinition() const
            {
                return pp_.getProblemDefinition();
            }

            /** \brief Get the problem definition used */
            base::ProblemDefinitionPtr &getProblemDefinition()
            {
                return pp_.getProblemDefinition();
            }

            /** \brief Try to solve the specified problem within a \e solveTime seconds, using at most \e nthreads
               threads. If
                more than \e maxSol solutions are generated, stop generating more. */
            base::PlannerStatus solve(double solveTime, unsigned int maxSol = 10, unsigned int nthreads = 1);

        protected:
            /** \brief Instance of parallel planning to use for computing solutions in parallel */
            ParallelPlan pp_;

            /** \brief The set of planners to be used */
            std::vector<base::PlannerPtr> planners_;
        };
    }
}
#endif
