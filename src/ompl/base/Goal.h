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
#include "ompl/util/ClassForward.h"
#include "ompl/base/GoalTypes.h"
#include "ompl/util/Console.h"
#include <iostream>
#include <boost/concept_check.hpp>
#include <vector>

namespace ompl
{
    namespace base
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::Goal */
        OMPL_CLASS_FORWARD(Goal);
        /// @endcond

        /** \class ompl::base::GoalPtr
            \brief A shared pointer wrapper for ompl::base::Goal */

        /** \brief Abstract definition of goals.*/
        class Goal
        {
        public:
            // non-copyable
            Goal(const Goal &) = delete;
            Goal &operator=(const Goal &) = delete;

            /** \brief Constructor. The goal must always know the space information it is part of */
            Goal(SpaceInformationPtr si);

            /** \brief Destructor.*/
            virtual ~Goal() = default;

            /** \brief Cast this instance to a desired type. */
            template <class T>
            T *as()
            {
                /** \brief Make sure the type we are casting to is indeed a goal */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T *, Goal *>));

                return static_cast<T *>(this);
            }

            /** \brief Cast this instance to a desired type. */
            template <class T>
            const T *as() const
            {
                /** \brief Make sure the type we are casting to is indeed a goal */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T *, Goal *>));

                return static_cast<const T *>(this);
            }

            /** \brief Return the goal type */
            GoalType getType() const
            {
                return type_;
            }

            /** \brief Check if this goal can be cast to a particular goal type */
            bool hasType(GoalType type) const
            {
                return (type_ & type) == type;
            }

            /** \brief Get the space information this goal is for */
            const SpaceInformationPtr &getSpaceInformation() const
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

            /** \brief Print information about the goal */
            virtual void print(std::ostream &out = std::cout) const;

        protected:
            /** \brief Goal type */
            GoalType type_;

            /** \brief The space information for this goal */
            SpaceInformationPtr si_;
        };
    }
}

#endif
