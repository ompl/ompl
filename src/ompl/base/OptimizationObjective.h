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

#ifndef OMPL_BASE_OPTIMIZATION_OBJECTIVE_
#define OMPL_BASE_OPTIMIZATION_OBJECTIVE_

#include "ompl/base/State.h"
#include "ompl/base/Path.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/util/ClassForward.h"
#include <boost/noncopyable.hpp>
#include <limits>

namespace ompl
{
    namespace base
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::OptimizationObjective */
        ClassForward(OptimizationObjective);
        /// @endcond

        /** \class ompl::base::OptimizationObjectivePtr
            \brief A boost shared pointer wrapper for ompl::base::OptimizationObjective */

        /** \brief Abstract definition of optimization objectives. */
        class OptimizationObjective : private boost::noncopyable
        {
        public:
            /** \brief Constructor. The objective must always know the space information it is part of */
            OptimizationObjective(const SpaceInformationPtr &si);

            virtual ~OptimizationObjective(void)
            {
            }
          
            /** \brief Get the description of this optimization objective */
            const std::string& getDescription(void) const
            {
                return description_;
            }

            /** \brief Verify that our objective is satisfied already and we can stop planning (it the cost is \e totalObjectiveCost) */
            virtual bool isSatisfied(double totalObjectiveCost) const = 0;

            /** \brief Get the cost that corresponds to the motion segment between \e s1 and \e s2 */
            virtual double getIncrementalCost(const State *s1, const State *s2) const = 0;

            /** \brief Get the cost that corresponds to combining the costs \e a and \e b */
            virtual double combineObjectiveCosts(double a, double b) const = 0;

            /** \brief Get the cost that corresponds to the sequence of motion segments defined by states. */
            virtual double getCost(const PathPtr &path) const;

        protected:
            /** \brief The space information for this objective */
            SpaceInformationPtr si_;

            /** \brief The description of this optimization objective */
            std::string description_;
        };

        /** \brief Representation of optimization objectives that are additive (we sum the values of the objective costs)
            and that are met as long as an upper bound is satisfied */
        class BoundedAdditiveOptimizationObjective : public OptimizationObjective
        {
        public:

            /** \brief Constructor. The objective must always know the space information it is part of */
            BoundedAdditiveOptimizationObjective(const SpaceInformationPtr &si, double maximumUpperBound) :
                OptimizationObjective(si),
                maximumUpperBound_(maximumUpperBound)
            {
            }
          
            /** \brief Get the maximum upper bound for the objective cost that is to be accepted as satisfactory */
            double getMaximumUpperBound(void) const
            {
                return maximumUpperBound_;
            }

            /** \brief Set the maximum upper bound for the objective cost that is to be accepted as satisfactory.
                The default value is usually infinity. */
            void setMaximumUpperBound(double maximumUpperBound)
            {
                maximumUpperBound_ = maximumUpperBound;
            }

            virtual bool isSatisfied(double totalObjectiveCost) const;

            virtual double combineObjectiveCosts(double a, double b) const;

        protected:

            /** \brief The maximum upper bound for the objective cost that is to be accepted as satisfactory */
            double  maximumUpperBound_;
        };
    
          
        class PathLengthOptimizationObjective : public BoundedAdditiveOptimizationObjective
        {   
        public:
      
            /** \brief Constructor. The objective must always know the space information it is part of */
            PathLengthOptimizationObjective(const SpaceInformationPtr &si, double maximumPathLength = std::numeric_limits<double>::infinity());

            virtual double getIncrementalCost(const State *s1, const State *s2) const;
        };

        class StateCostOptimizationObjective : public BoundedAdditiveOptimizationObjective
        {   
        public:
      
            /** \brief Constructor. The objective must always know the space information it is part of */
            StateCostOptimizationObjective(const SpaceInformationPtr &si, double maximumCostSum = std::numeric_limits<double>::infinity());

            virtual double getIncrementalCost(const State *s1, const State *s2) const;
        };
      
    }
}

#endif
