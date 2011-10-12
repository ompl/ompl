/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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

#ifndef OMPL_BASE_GOAL_
#define OMPL_BASE_GOAL_

#include "ompl/base/State.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/Path.h"
#include "ompl/util/ClassForward.h"
#include "ompl/base/GoalTypes.h"
#include <iostream>
#include <boost/noncopyable.hpp>
#include <boost/concept_check.hpp>
#include <vector>

namespace ompl
{
    namespace base
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::Goal */
        ClassForward(Goal);
        /// @endcond

        /** \class ompl::base::GoalPtr
            \brief A boost shared pointer wrapper for ompl::base::Goal */

        /** \brief Representation of a solution to a planning problem */
        struct PlannerSolution
        {
            /** \brief Construct a solution that consists of a \e path and its attributes (whether it is \e approximate and the \e difference to the desired goal) */
            PlannerSolution(const PathPtr &path, bool approximate = false, double difference = -1.0) :
                index_(-1), path_(path), approximate_(approximate), difference_(difference)
            {
            }

            /** \brief When multiple solutions are found, each is given a number starting at 0, so that the order in which the solutions was found can be retrieved. */
            int     index_;

            /** \brief Solution path */
            PathPtr path_;

            /** \brief True if goal was not achieved, but an approximate solution was found */
            bool    approximate_;

            /** \brief The achieved difference between the found solution and the desired goal */
            double  difference_;
        };

        /// @cond IGNORE
        ClassForward(PlannerSolutionSet);
        /// @endcond


        /** \brief Abstract definition of goals. Will contain solutions, if found */
        class Goal : private boost::noncopyable
        {
        public:

            /** \brief Constructor. The goal must always know the space information it is part of */
            Goal(const SpaceInformationPtr &si);

            /** \brief Destructor. Clears the solution as well */
            virtual ~Goal(void)
            {
            }

            /** \brief Cast this instance to a desired type. */
            template<class T>
            T* as(void)
            {
                /** \brief Make sure the type we are casting to is indeed a goal */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T*, Goal*>));

                return static_cast<T*>(this);
            }

            /** \brief Cast this instance to a desired type. */
            template<class T>
            const T* as(void) const
            {
                /** \brief Make sure the type we are casting to is indeed a goal */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T*, Goal*>));

                return static_cast<const T*>(this);
            }

            /** \brief Return the goal type */
            GoalType getType(void) const
            {
                return type_;
            }

            /** \brief Check if this goal can be cast to a particular goal type */
            bool hasType(GoalType type) const
            {
                return (type_ & type) == type;
            }

            /** \brief Get the space information this goal is for */
            const SpaceInformationPtr& getSpaceInformation(void) const
            {
                return si_;
            }

            /** \brief Return true if the state satisfies the goal
             *  constraints. */
            virtual bool isSatisfied(const State *st) const = 0;

            /** \brief Return true if the state satisfies the goal
             *  constraints and compute the distance between the state
             *  given as argument and the goal (even if the goal is
             *  not satisfied). This distance can be an
             *  approximation. It can even be set to a constant, if
             *  such a computation is not possible.
             *  \param st the state to check for validity
             *  \param distance location at which distance to goal will be stored
             *  \note The default implementation sets the distance to a constant.
             *  \note If this function returns true,
             *  isStartGoalPairValid() need not be called. */
            virtual bool isSatisfied(const State *st, double *distance) const;

            /** \brief Return true if the state satisfies the goal
             *  constraints and the path length is less than the
             *  desired maximum length.  This call also computes the
             *  distance between the state given as argument and the
             *  goal.
             *  \param st the state to check for validity
             *  \param pathLength the length of the path that leads to \e st
             *  \param distance location at which distance to goal will be stored
             */
            bool isSatisfied(const State *st, double pathLength, double *distance) const;

            /** \brief Since there can be multiple starting states
                (and multiple goal states) it is possible certain
                pairs are not to be allowed. By default we however
                assume all such pairs are allowed. Note: if this
                function returns true, isSatisfied() need not be
                called. */
            virtual bool isStartGoalPairValid(const State * /* start */, const State * /* goal */) const
            {
                return true;
            }

            /** \brief Get the maximum length allowed for a solution path */
            double getMaximumPathLength(void) const
            {
                return maximumPathLength_;
            }

            /** \brief Set the maximum length allowed for a solution
                path. This value is checked only in the version of
                isSatisfied() that takes the path length as argument
                or by isPathLengthSatisfied(). The default maximal
                path length is infinity. */
            void setMaximumPathLength(double maximumPathLength)
            {
                maximumPathLength_ = maximumPathLength;
            }

            /** \brief Check if \e pathLength is smaller than the value returned by getMaximumPathLength() */
            bool isPathLengthSatisfied(double pathLength) const
            {
                return pathLength <= maximumPathLength_;
            }

            /** \brief Returns true if a solution path has been found (could be approximate) */
            bool isAchieved(void) const;

            /** \brief Return true if the top found solution is
                approximate (does not actually reach the desired goal,
                but hopefully is closer to it) */
            bool isApproximate(void) const;

            /** \brief Get the distance to the desired goal for the top solution. Return -1.0 if there are no solutions available. */
            double getDifference(void) const;

            /** \brief Return the top solution path, if one is found. The top path is the shortest one that was found, preference being given to solutions that are not approximate.

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

            /** \brief Print information about the goal */
            virtual void print(std::ostream &out = std::cout) const;

        protected:

            /** \brief Goal type */
            GoalType                     type_;

            /** \brief The space information for this goal */
            SpaceInformationPtr          si_;

            /** \brief The maximum length allowed for the solution path */
            double                       maximumPathLength_;

        private:

            /** \brief The set of solutions computed for this goal (maintains an array of PlannerSolution) */
            PlannerSolutionSetPtr        solutions_;
        };

    }
}

#endif
