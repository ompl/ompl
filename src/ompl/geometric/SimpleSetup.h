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

#ifndef OMPL_GEOMETRIC_SIMPLE_SETUP_
#define OMPL_GEOMETRIC_SIMPLE_SETUP_

#include "ompl/base/Planner.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/ProblemDefinition.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/geometric/PathSimplifier.h"
#include "ompl/util/Console.h"
#include "ompl/util/Exception.h"

namespace ompl
{
    namespace geometric
    {
        /// @cond IGNORE
        OMPL_CLASS_FORWARD(SimpleSetup);
        /// @endcond

        /** \class ompl::geometric::SimpleSetupPtr
            \brief A shared pointer wrapper for ompl::geometric::SimpleSetup */

        /** \brief Create the set of classes typically needed to solve a
            geometric problem */
        class SimpleSetup
        {
        public:
            /** \brief Constructor needs the state space used for planning. */
            explicit SimpleSetup(const base::SpaceInformationPtr &si);

            /** \brief Constructor needs the state space used for planning. */
            explicit SimpleSetup(const base::StateSpacePtr &space);

            virtual ~SimpleSetup() = default;

            /** \brief Get the current instance of the space information */
            const base::SpaceInformationPtr &getSpaceInformation() const
            {
                return si_;
            }

            /** \brief Get the current instance of the problem definition */
            const base::ProblemDefinitionPtr &getProblemDefinition() const
            {
                return pdef_;
            }

            /** \brief Get the current instance of the problem definition */
            base::ProblemDefinitionPtr &getProblemDefinition()
            {
                return pdef_;
            }

            /** \brief Get the current instance of the state space */
            const base::StateSpacePtr &getStateSpace() const
            {
                return si_->getStateSpace();
            }

            /** \brief Get the current instance of the state validity checker */
            const base::StateValidityCheckerPtr &getStateValidityChecker() const
            {
                return si_->getStateValidityChecker();
            }

            /** \brief Get the current goal definition */
            const base::GoalPtr &getGoal() const
            {
                return pdef_->getGoal();
            }

            /** \brief Get the current planner */
            const base::PlannerPtr &getPlanner() const
            {
                return planner_;
            }

            /** \brief Get the planner allocator */
            const base::PlannerAllocator &getPlannerAllocator() const
            {
                return pa_;
            }

            /** \brief Get the path simplifier */
            const PathSimplifierPtr &getPathSimplifier() const
            {
                return psk_;
            }

            /** \brief Get the path simplifier */
            PathSimplifierPtr &getPathSimplifier()
            {
                return psk_;
            }

            /** \brief Get the optimization objective to use */
            const base::OptimizationObjectivePtr &getOptimizationObjective() const
            {
                return pdef_->getOptimizationObjective();
            }

            /** \brief Return true if a solution path is available (previous call to solve() was successful) and the
             * solution is exact (not approximate) */
            bool haveExactSolutionPath() const
            {
                return pdef_->hasExactSolution();
            }

            /** \brief Return true if a solution path is available (previous call to solve() was successful). The
             * solution may be approximate. */
            bool haveSolutionPath() const
            {
                return pdef_->hasSolution();
            }

            /** \brief Get the best solution's planer name. Throw an exception if no solution is available */
            const std::string getSolutionPlannerName() const;

            /** \brief Get the solution path. Throw an exception if no solution is available */
            PathGeometric &getSolutionPath() const;

            /** \brief Get information about the exploration data structure the motion planner used. */
            void getPlannerData(base::PlannerData &pd) const;

            /** \brief Set the state validity checker to use */
            void setStateValidityChecker(const base::StateValidityCheckerPtr &svc)
            {
                si_->setStateValidityChecker(svc);
            }

            /** \brief Set the state validity checker to use */
            void setStateValidityChecker(const base::StateValidityCheckerFn &svc)
            {
                si_->setStateValidityChecker(svc);
            }

            /** \brief Set the optimization objective to use */
            void setOptimizationObjective(const base::OptimizationObjectivePtr &optimizationObjective)
            {
                pdef_->setOptimizationObjective(optimizationObjective);
            }

            /** \brief Set the start and goal states to use. */
            void setStartAndGoalStates(const base::ScopedState<> &start, const base::ScopedState<> &goal,
                                       double threshold = std::numeric_limits<double>::epsilon());

            /** \brief Add a starting state for planning. This call is not
                needed if setStartAndGoalStates() has been called. */
            void addStartState(const base::ScopedState<> &state)
            {
                pdef_->addStartState(state);
            }

            /** \brief Clear the currently set starting states */
            void clearStartStates()
            {
                pdef_->clearStartStates();
            }

            /** \brief Clear the currently set starting states and add \e state as the starting state */
            void setStartState(const base::ScopedState<> &state)
            {
                clearStartStates();
                addStartState(state);
            }

            /** \brief A simple form of setGoal(). The goal will be an instance of ompl::base::GoalState */
            void setGoalState(const base::ScopedState<> &goal,
                              double threshold = std::numeric_limits<double>::epsilon());

            /** \brief Set the goal for planning. This call is not
                needed if setStartAndGoalStates() has been called. */
            void setGoal(const base::GoalPtr &goal);

            /** \brief Set the planner to use. If the planner is not
                set, an attempt is made to use the planner
                allocator. If no planner allocator is available
                either, a default planner is set. */
            void setPlanner(const base::PlannerPtr &planner)
            {
                if (planner && planner->getSpaceInformation().get() != si_.get())
                    throw Exception("Planner instance does not match space information");
                planner_ = planner;
                configured_ = false;
            }

            /** \brief Set the planner allocator to use. This is only
                used if no planner has been set. This is optional -- a default
                planner will be used if no planner is otherwise specified. */
            void setPlannerAllocator(const base::PlannerAllocator &pa)
            {
                pa_ = pa;
                planner_.reset();
                configured_ = false;
            }

            /** \brief Run the planner for up to a specified amount of time (default is 1 second) */
            virtual base::PlannerStatus solve(double time = 1.0);

            /** \brief Run the planner until \e ptc becomes true (at most) */
            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            /** \brief Return the status of the last planning attempt */
            base::PlannerStatus getLastPlannerStatus() const
            {
                return lastStatus_;
            }

            /** \brief Get the amount of time (in seconds) spent during the last planning step */
            double getLastPlanComputationTime() const
            {
                return planTime_;
            }

            /** \brief Get the amount of time (in seconds) spend during the last path simplification step */
            double getLastSimplificationTime() const
            {
                return simplifyTime_;
            }

            /** \brief Attempt to simplify the current solution path. Spent at most \e duration seconds in the
               simplification process.
                If \e duration is 0 (the default), a default simplification procedure is executed. */
            void simplifySolution(double duration = 0.0);

            /** \brief Attempt to simplify the current solution path. Stop computation when \e ptc becomes true at the
             * latest. */
            void simplifySolution(const base::PlannerTerminationCondition &ptc);

            /** \brief Clear all planning data. This only includes
                data generated by motion plan computation. Planner
                settings, start & goal states are not affected. */
            virtual void clear();

            /** \brief Print information about the current setup */
            virtual void print(std::ostream &out = std::cout) const;

            /** \brief This method will create the necessary classes
                for planning. The solve() method will call this
                function automatically. */
            virtual void setup();

        protected:
            /// The created space information
            base::SpaceInformationPtr si_;

            /// The created problem definition
            base::ProblemDefinitionPtr pdef_;

            /// The maintained planner instance
            base::PlannerPtr planner_;

            /// The optional planner allocator
            base::PlannerAllocator pa_;

            /// The instance of the path simplifier
            PathSimplifierPtr psk_;

            /// Flag indicating whether the classes needed for planning are set up
            bool configured_;

            /// The amount of time the last planning step took
            double planTime_;

            /// The amount of time the last path simplification step took
            double simplifyTime_;

            /// The status of the last planning request
            base::PlannerStatus lastStatus_;
        };
    }
}
#endif
