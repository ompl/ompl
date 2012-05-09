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

#ifndef OMPL_BASE_PROBLEM_DEFINITION_
#define OMPL_BASE_PROBLEM_DEFINITION_

#include "ompl/base/State.h"
#include "ompl/base/Goal.h"
#include "ompl/base/Path.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/util/Console.h"
#include "ompl/util/ClassForward.h"
#include "ompl/base/ScopedState.h"

#include <vector>
#include <cstdlib>
#include <iostream>
#include <limits>

#include <boost/noncopyable.hpp>

namespace ompl
{
    namespace base
    {

        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::ProblemDefinition */
        ClassForward(ProblemDefinition);
        /// @endcond

        /** \class ompl::base::ProblemDefinitionPtr
            \brief A boost shared pointer wrapper for ompl::base::ProblemDefinition */

        /** \brief Representation of a solution to a planning problem */
        struct PlannerSolution
        {
            /** \brief Construct a solution that consists of a \e path and its attributes (whether it is \e approximate and the \e difference to the desired goal) */
            PlannerSolution(const PathPtr &path, bool approximate = false, double difference = -1.0) :
                index_(-1), path_(path), length_(path->length()), approximate_(approximate), difference_(difference)
            {
            }

            /** \brief Return true if two solutions are the same */
            bool operator==(const PlannerSolution& p) const
            {
                return path_ == p.path_;
            }

            /** \brief Define a ranking for solutions */
            bool operator<(const PlannerSolution &b) const
            {
                if (!approximate_ && b.approximate_)
                    return true;
                if (approximate_ && !b.approximate_)
                    return false;
                if (approximate_ && b.approximate_)
                    return difference_ < b.difference_;
                return length_ < b.length_;
            }

            /** \brief When multiple solutions are found, each is given a number starting at 0, so that the order in which the solutions was found can be retrieved. */
            int     index_;

            /** \brief Solution path */
            PathPtr path_;

            /** \brief For efficiency reasons, keep the length of the path as well */
            double  length_;

            /** \brief True if goal was not achieved, but an approximate solution was found */
            bool    approximate_;

            /** \brief The achieved difference between the found solution and the desired goal */
            double  difference_;
        };

        /** \brief Definition of a problem to be solved. This includes
            the start state(s) for the system and a goal specification */
        class ProblemDefinition : private boost::noncopyable
        {
        public:

            /** \brief Create a problem definition given the SpaceInformation it is part of */
            ProblemDefinition(const SpaceInformationPtr &si);

            virtual ~ProblemDefinition(void)
            {
                clearStartStates();
            }

            /** \brief Get the space information this problem definition is for */
            const SpaceInformationPtr& getSpaceInformation(void) const
            {
                return si_;
            }

            /** \brief Add a start state. The state is copied. */
            void addStartState(const State *state)
            {
                startStates_.push_back(si_->cloneState(state));
            }

            /** \copydoc addStartState() */
            void addStartState(const ScopedState<> &state)
            {
                startStates_.push_back(si_->cloneState(state.get()));
            }

            /** \brief Check whether a specified starting state is
                already included in the problem definition and
                optionally return the index of that starting state */
            bool hasStartState(const State *state, unsigned int *startIndex = NULL);

            /** \brief Clear all start states (memory is freed) */
            void clearStartStates(void)
            {
                for (unsigned int i = 0 ; i < startStates_.size() ; ++i)
                    si_->freeState(startStates_[i]);
                startStates_.clear();
            }

            /** \brief Returns the number of start states */
            unsigned int getStartStateCount(void) const
            {
                return startStates_.size();
            }

            /** \brief Returns a specific start state */
            const State* getStartState(unsigned int index) const
            {
                return startStates_[index];
            }

            /** \copydoc getStartState() */
            State* getStartState(unsigned int index)
            {
                return startStates_[index];
            }

            /** \brief Set the goal. */
            void setGoal(const GoalPtr &goal)
            {
                goal_ = goal;
            }

            /** \brief Clear the goal. Memory is freed. */
            void clearGoal(void)
            {
                goal_.reset();
            }

            /** \brief Return the current goal */
            const GoalPtr& getGoal(void) const
            {
                return goal_;
            }

            /** \brief Get all the input states. This includes start
                states and states that are part of goal regions that
                can be casted as ompl::base::GoalState or
                ompl::base::GoalStates. */
            void getInputStates(std::vector<const State*> &states) const;

            /** \brief In the simplest case possible, we have a single
                starting state and a single goal state.

                This function simply configures the problem definition
                using these states (performs the needed calls to
                addStartState(), creates an instance of
                ompl::base::GoalState and calls setGoal() on it. */
            void setStartAndGoalStates(const State *start, const State *goal, const double threshold = std::numeric_limits<double>::epsilon());

            /** \brief A simple form of setting the goal. This is called by setStartAndGoalStates(). A more general form is setGoal() */
            void setGoalState(const State *goal, const double threshold = std::numeric_limits<double>::epsilon());

            /** \copydoc setStartAndGoalStates() */
            void setStartAndGoalStates(const ScopedState<> &start, const ScopedState<> &goal, const double threshold = std::numeric_limits<double>::epsilon())
            {
                setStartAndGoalStates(start.get(), goal.get(), threshold);
            }

            /** \copydoc setGoalState() */
            void setGoalState(const ScopedState<> &goal, const double threshold = std::numeric_limits<double>::epsilon())
            {
                setGoalState(goal.get(), threshold);
            }

            /** \brief A problem is trivial if a given starting state already
                in the goal region, so we need no motion planning. startID
                will be set to the index of the starting state that
                satisfies the goal. The distance to the goal can
                optionally be returned as well. */
            bool isTrivial(unsigned int *startIndex = NULL, double *distance = NULL) const;

            /** \brief Check if a straight line path is valid. If it
                is, return an instance of a path that represents the
                straight line.

                \note When planning under geometric constraints, this
                works only if the goal region can be sampled. If the
                goal region cannot be sampled, this call is equivalent
                to calling isTrivial()

                \note When planning under differential constraints,
                the system is propagated forward in time using the
                null control. */
            PathPtr isStraightLinePathValid(void) const;

            /** \brief Many times the start or goal state will barely touch an obstacle. In this case, we may want to automatically
              * find a nearby state that is valid so motion planning can be performed. This function enables this behaviour.
              * The allowed distance for both start and goal states is specified. The number of attempts
              * is also specified. Returns true if all states are valid after completion. */
            bool fixInvalidInputStates(double distStart, double distGoal, unsigned int attempts);

            /** \brief Returns true if a solution path has been found (could be approximate) */
            bool hasSolution(void) const;

            /** \brief Return true if the top found solution is
                approximate (does not actually reach the desired goal,
                but hopefully is closer to it) */
            bool hasApproximateSolution(void) const;

            /** \brief Get the distance to the desired goal for the top solution. Return -1.0 if there are no solutions available. */
            double getSolutionDifference(void) const;

            /** \brief Return the top solution path, if one is found. The top path is the shortest
                 one that was found, preference being given to solutions that are not approximate.

                This will need to be casted into the specialization computed by the planner */
            PathPtr getSolutionPath(void) const;

            /** \brief Add a solution path in a thread-safe manner. Multiple solutions can be set for a goal.
                If a solution does not reach the desired goal it is considered approximate.
                Optionally, the distance between the desired goal and the one actually achieved is set by \e difference.
            */
            void addSolutionPath(const PathPtr &path, bool approximate = false, double difference = -1.0) const;

            /** \brief Get the number of solutions already found */
            std::size_t getSolutionCount(void) const;

            /** \brief Get all the solution paths available for this goal */
            std::vector<PlannerSolution> getSolutions(void) const;

            /** \brief Forget the solution paths (thread safe). Memory is freed. */
            void clearSolutionPaths(void) const;

            /** \brief Print information about the start and goal states */
            void print(std::ostream &out = std::cout) const;

        protected:

            /** \brief Helper function for fixInvalidInputStates(). Attempts to fix an individual state */
            bool fixInvalidInputState(State *state, double dist, bool start, unsigned int attempts);

            /** \brief The space information this problem definition is for */
            SpaceInformationPtr  si_;

            /** \brief The set of start states */
            std::vector<State*>  startStates_;

            /** \brief The goal representation */
            GoalPtr              goal_;

            /** \brief Interface for console output */
            msg::Interface       msg_;

        private:

            /// @cond IGNORE
            ClassForward(PlannerSolutionSet);
            /// @endcond

            /** \brief The set of solutions computed for this goal (maintains an array of PlannerSolution) */
            PlannerSolutionSetPtr        solutions_;
        };
    }
}

#endif
