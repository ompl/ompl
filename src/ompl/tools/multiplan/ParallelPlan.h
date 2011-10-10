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
#include <boost/thread.hpp>

namespace ompl
{

    /** \brief Forward declaration of ompl::ParallelPlan */
    ClassForward(ParallelPlan);

    /// @cond IGNORE
    namespace geometric
    {
        ClassForward(PathHybridization);
    }
    /// @endcond

    class ParallelPlan
    {
    public:

        /** \brief Create an instance for a specified space information */
        ParallelPlan(const base::ProblemDefinitionPtr &pdef);

        virtual ~ParallelPlan(void);

        /** \brief Add a planner to use. */
        void addPlanner(const base::PlannerPtr &planner);

        /** \brief Add a planner allocator to use. */
        void addPlannerAllocator(const base::PlannerAllocator &pa);

        /** \brief Clear the set of planners to be benchmarked */
        void clearPlanners(void);

        /** \brief Enable or disable hybridization of solution paths (using ompl::geometric::PathHybridization) */
        void setHybridization(bool flag);

        /** \brief Call Planner::solve() for all planners, in parallel */
        bool solve(double solveTime);

        /** \brief Call Planner::solve() for all planners, in parallel */
        bool solve(const base::PlannerTerminationCondition &ptc);

    protected:

        /** \brief Run the planner and call ompl::base::PlannerTerminationCondition::terminate() for the other planners once a first solution is found */
        void solveOne(base::Planner *planner, const base::PlannerTerminationCondition *ptc);

        /** \brief Run the planner and collect the solutions. This function is only called if hybridize_ is true. */
        void solveMore(base::Planner *planner, const base::PlannerTerminationCondition *ptc);

        /** \brief The space information this path simplifier uses */
        base::ProblemDefinitionPtr      pdef_;

        /** \brief The set of planners to be used */
        std::vector<base::PlannerPtr>   planners_;

        /** \brief Flag indicating whether path hybridization is enabled */
        bool                            hybridize_;

        /** \brief Minimum number of solutions to find before attempting path hybridization */
        std::size_t                     minSolCount_;

        /** \brief Maximum number of solutions to find before stopping the hybridization */
        std::size_t                     maxSolCount_;

        /** \brief The instance of the class that performs path hybridization */
        geometric::PathHybridizationPtr phybrid_;

        boost::mutex                    phlock_;

        /** \brief Interface for console output */
        msg::Interface                  msg_;
    };
}

#endif
