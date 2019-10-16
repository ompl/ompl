/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage, Inc.
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

#ifndef OMPL_TOOLS_MULTIPLAN_PARALLEL_PLAN_
#define OMPL_TOOLS_MULTIPLAN_PARALLEL_PLAN_

#include "ompl/base/Planner.h"
#include "ompl/geometric/PathGeometric.h"
#include <mutex>

namespace ompl
{
    /// @cond IGNORE
    namespace geometric
    {
        OMPL_CLASS_FORWARD(PathHybridization);
    }
    /// @endcond

    namespace tools
    {
        /// @cond IGNORE
        OMPL_CLASS_FORWARD(ParallelPlan);
        /// @endcond

        /** \brief This is a utility that allows executing multiple
            planners in parallel, until one or more find a
            solution. Optionally, the results are automatically hybridized
            using ompl::geometric::PathHybridization. Between calls to
            solve(), the set of known solutions (maintained by
            ompl::base::Goal) are not cleared, and neither is the
            hybridization datastructure.*/
        class ParallelPlan
        {
        public:
            /** \brief Create an instance for a specified space information */
            ParallelPlan(const base::ProblemDefinitionPtr &pdef);

            virtual ~ParallelPlan();

            /** \brief Add a planner to use. */
            void addPlanner(const base::PlannerPtr &planner);

            /** \brief Add a planner allocator to use. */
            void addPlannerAllocator(const base::PlannerAllocator &pa);

            /** \brief Clear the set of paths recorded for hybrididzation */
            void clearHybridizationPaths();

            /** \brief Clear the set of planners to be executed */
            void clearPlanners();

            /** \brief Get the problem definition used */
            const base::ProblemDefinitionPtr &getProblemDefinition() const
            {
                return pdef_;
            }

            /** \brief Get the problem definition used */
            base::ProblemDefinitionPtr &getProblemDefinition()
            {
                return pdef_;
            }

            /** \brief Call Planner::solve() for all planners, in parallel, each planner running for at most \e
               solveTime seconds.
                If \e hybridize is false, when the first solution is found, the rest of the planners are stopped as
               well.
                If \e hybridize is true, all planners are executed until termination and the obtained solution paths are
               hybridized. */
            base::PlannerStatus solve(double solveTime, bool hybridize = true);

            /** \brief Call Planner::solve() for all planners, in parallel, until the termination condition \e ptc
               becomes true.
                If \e hybridize is false, when the first solution is found, the rest of the planners are stopped as
               well.
                If \e hybridize is true, all planners are executed until termination and the obtained solution paths are
               hybridized. */
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc, bool hybridize = true);

            /** \brief Call Planner::solve() for all planners, in parallel, each planner running for at most \e
               solveTime seconds.
                If \e hybridize is false, when \e minSolCount new solutions are found (added to the set of solutions
               maintained by ompl::base::Goal), the rest of the planners are stopped as well.
                If \e hybridize is true, all planners are executed until termination or until \e maxSolCount new
               solutions were obtained.
                While \e hybridize is true, if \e minSolCount or more solution paths are available, they are hybridized.
               */
            base::PlannerStatus solve(double solveTime, std::size_t minSolCount, std::size_t maxSolCount,
                                      bool hybridize = true);

            /** \brief Call Planner::solve() for all planners, in parallel, until the termination condition \e ptc
               becomes true.
                If \e hybridize is false, when \e minSolCount new solutions are found (added to the set of solutions
               maintained by ompl::base::Goal), the rest of the planners are stopped as well.
                If \e hybridize is true, all planners are executed until termination or until \e maxSolCount new
               solutions were obtained.
                While \e hybridize is true, if \e minSolCount or more solution paths are available, they are hybridized.
               */
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc, std::size_t minSolCount,
                                      std::size_t maxSolCount, bool hybridize = true);

        protected:
            /** \brief Run the planner and call ompl::base::PlannerTerminationCondition::terminate() for the other
             * planners once a first solution is found */
            void solveOne(base::Planner *planner, std::size_t minSolCount,
                          const base::PlannerTerminationCondition *ptc);

            /** \brief Run the planner and collect the solutions. This function is only called if hybridize_ is true. */
            void solveMore(base::Planner *planner, std::size_t minSolCount, std::size_t maxSolCount,
                           const base::PlannerTerminationCondition *ptc);

            /** \brief The problem definition used */
            base::ProblemDefinitionPtr pdef_;

            /** \brief The set of planners to be used */
            std::vector<base::PlannerPtr> planners_;

            /** \brief The instance of the class that performs path hybridization */
            geometric::PathHybridizationPtr phybrid_;

            /** \brief Lock for phybrid_ */
            std::mutex phlock_;

        private:
            /** \brief Number of solutions found during a particular run */
            unsigned int foundSolCount_;

            /** \brief Lock for phybrid_ */
            std::mutex foundSolCountLock_;
        };
    }
}

#endif
