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

/* Author: Luis G. Torres, Ioan Sucan */

#ifndef OMPL_BASE_OPTIMIZATION_OBJECTIVE_
#define OMPL_BASE_OPTIMIZATION_OBJECTIVE_

#include "ompl/base/Cost.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/util/ClassForward.h"
#include <boost/noncopyable.hpp>
#include <boost/concept_check.hpp>

#include <iostream>

namespace ompl
{
    namespace base
    {
        class Goal;

        /** \brief The definition of a function which returns an admissible estimate of the optimal path cost from a given state to a goal. */
        typedef boost::function<Cost (const State*, const Goal*)> CostToGoHeuristic;

        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::OptimizationObjective */
        OMPL_CLASS_FORWARD(OptimizationObjective);
        /// @endcond

        /** \class ompl::base::OptimizationObjectivePtr
            \brief A boost shared pointer wrapper for ompl::base::OptimizationObjective */

        /** \brief Abstract definition of optimization objectives.

            \note This implementation has greatly benefited from discussions with Kris Hauser */
        class OptimizationObjective : private boost::noncopyable
        {
        public:
            /** \brief Constructor. The objective must always know the space information it is part of. The cost threshold for objective satisfaction defaults to 0.0. */
            OptimizationObjective(const SpaceInformationPtr &si);

            virtual ~OptimizationObjective()
            {
            }

            /** \brief Get the description of this optimization objective */
            const std::string& getDescription() const;

            /** \brief Verify that our objective is satisfied already and we can stop planning */
            virtual bool isSatisfied(Cost c) const;

            /** \brief Returns the cost threshold currently being checked for objective satisfaction */
            Cost getCostThreshold() const;

            /** \brief Set the cost threshold for objective satisfaction. When a path is found with a cost better than the cost threshold, the objective is considered satisfied. */
            void setCostThreshold(Cost c);

            /** \brief Check whether the the cost \e c1 is considered better than the cost \e c2. By default, this returns true only if c1 is less by at least some threshold amount, for numerical robustness. */
            virtual bool isCostBetterThan(Cost c1, Cost c2) const;

            /** \brief Evaluate a cost map defined on the state space at a state \e s. */
            virtual Cost stateCost(const State *s) const = 0;

            /** \brief Get the cost that corresponds to the motion segment between \e s1 and \e s2 */
            virtual Cost motionCost(const State *s1, const State *s2) const = 0;

            /** \brief Get the cost that corresponds to combining the costs \e c1 and \e c2. Default implementation defines this combination as an addition. */
            virtual Cost combineCosts(Cost c1, Cost c2) const;

            /** \brief Get the identity cost value. The identity cost value is the cost c_i such that, for all costs c, combineCosts(c, c_i) = combineCosts(c_i, c) = c. In other words, combining a cost with the identity cost does not change the original cost. By default, a cost with the value 0.0 is returned. It's very important to override this with the proper identity value for your optimization objectives, or else optimal planners may not work. */
            virtual Cost identityCost() const;

            /** \brief Get a cost which is greater than all other costs in this OptimizationObjective; required for use in Dijkstra/Astar. Defaults to returning the double value inf.*/
            virtual Cost infiniteCost() const;

            /** \brief Returns a cost value corresponding to starting at a state \e s. No optimal planners currently support this method. Defaults to returning the objective's identity cost. */
            virtual Cost initialCost(const State *s) const;

            /** \brief Returns a cost value corresponding to a path ending at a state \e s. No optimal planners currently support this method. Defaults to returning the objective's identity cost. */
            virtual Cost terminalCost(const State *s) const;

            /** \brief Check if this objective has a symmetric cost metric, i.e. motionCost(s1, s2) = motionCost(s2, s1). Default implementation returns whether the underlying state space has symmetric interpolation. */
            virtual bool isSymmetric() const;

            /** \brief Compute the average state cost of this objective by taking a sample of \e numStates states */
            virtual Cost averageStateCost(unsigned int numStates) const;

            /** \brief Set the cost-to-go heuristic function for this objective. The cost-to-go heuristic is a function which returns an admissible estimate of the optimal path cost from a given state to a goal, where "admissible" means that the estimated cost is always less than the true optimal cost. */
            void setCostToGoHeuristic(const CostToGoHeuristic& costToGo);

            /** \brief Uses a cost-to-go heuristic to calculate an admissible estimate of the optimal cost from a given state to a given goal. If no cost-to-go heuristic has been specified with setCostToGoHeuristic(), this function just returns the identity cost, which is sure to be an admissible heuristic if there are no negative costs. */
            Cost costToGo(const State *state, const Goal *goal) const;

            /** \brief Defines an admissible estimate on the optimal cost on the motion between states \e s1 and \e s2. An admissible estimate always undervalues the true optimal cost of the motion. Used by some planners to speed up planning. The default implementation of this method returns this objective's identity cost, which is sure to be an admissible heuristic if there are no negative costs. */
            virtual Cost motionCostHeuristic(const State *s1, const State *s2) const;

            /** \brief Returns this objective's SpaceInformation. Needed for operators in MultiOptimizationObjective */
            const SpaceInformationPtr& getSpaceInformation() const;

            /** \brief Print information about this optimization objective */
            virtual void print(std::ostream &out) const;
        protected:
            /** \brief The space information for this objective */
            SpaceInformationPtr si_;

            /** \brief The description of this optimization objective */
            std::string         description_;

            /** \brief The cost threshold used for checking whether this objective has been satisfied during planning */
            Cost                threshold_;

            /** \brief The function used for returning admissible estimates on the optimal cost of the path between a given state and goal */
            CostToGoHeuristic   costToGoFn_;
        };

        /**
            \brief For use when goal region's distanceGoal() is
            equivalent to the cost-to-go of a state under the
            optimization objective. This function assumes that all states
            within the goal region's threshold have a cost-to-go of
            exactly zero. Note: \e goal is assumed to be of type
            ompl::base::GoalRegion
        */
        Cost goalRegionCostToGo(const State *state, const Goal *goal);

        /** \brief This class allows for the definition of multiobjective optimal planning problems. Objectives are added to this compound object, and motion costs are computed by taking a weighted sum of the individual objective costs. */
        class MultiOptimizationObjective : public OptimizationObjective
        {
        public:
            MultiOptimizationObjective(const SpaceInformationPtr &si);

            /** \brief Adds a new objective for this multiobjective. A weight must also be specified for specifying importance of this objective in planning. */
            void addObjective(const OptimizationObjectivePtr& objective,
                              double weight);

            /** \brief Returns the number of objectives that make up this multiobjective. */
            std::size_t getObjectiveCount() const;

            /** \brief Returns a specific objective from this multiobjective, where the individual objectives are in order of addition to the multiobjective, and \e idx is the zero-based index into this ordering. */
            const OptimizationObjectivePtr& getObjective(unsigned int idx) const;

            /** \brief Returns the weighing factor of a specific objective */
            double getObjectiveWeight(unsigned int idx) const;

            /** \brief Sets the weighing factor of a specific objective */
            void setObjectiveWeight(unsigned int idx, double weight);

            /** \brief This method "freezes" this multiobjective so that no more objectives can be added to it */
            void lock();

            /** \brief Returns whether this multiobjective has been locked from adding further objectives */
            bool isLocked() const;

            /** The default implementation of this method is to use
              addition to add up all the individual objectives' state cost
              values, where each individual value is scaled by its
              weight */
            virtual Cost stateCost(const State *s) const;

            /** The default implementation of this method is to use
              addition to add up all the individual objectives' motion
              cost values, where each individual value is scaled by
              its weight */
            virtual Cost motionCost(const State *s1, const State *s2) const;

        protected:

            /** \brief Defines a pairing of an objective and its weight */
            struct Component
            {
                Component(const OptimizationObjectivePtr& obj, double weight);
                OptimizationObjectivePtr objective;
                double weight;
            };

            /** \brief List of objective/weight pairs */
            std::vector<Component> components_;

            /** \brief Whether this multiobjective is locked from further additions */
            bool                   locked_;

            // Friend functions for operator overloads for easy multiobjective creation
            friend OptimizationObjectivePtr operator+(const OptimizationObjectivePtr &a,
                                                      const OptimizationObjectivePtr &b);

            friend OptimizationObjectivePtr operator*(double w, const OptimizationObjectivePtr &a);

            friend OptimizationObjectivePtr operator*(const OptimizationObjectivePtr &a, double w);
        };

        /** \brief Given two optimization objectives, returns a MultiOptimizationObjective that combines the two objectives with both weights equal to 1.0. */
        OptimizationObjectivePtr operator+(const OptimizationObjectivePtr &a,
                                           const OptimizationObjectivePtr &b);

        /** \brief Given a weighing factor and an optimization objective, returns a MultiOptimizationObjective containing only this objective weighted by the given weight */
        OptimizationObjectivePtr operator*(double w, const OptimizationObjectivePtr &a);

        /** \brief Given a weighing factor and an optimization objective, returns a MultiOptimizationObjective containing only this objective weighted by the given weight */
        OptimizationObjectivePtr operator*(const OptimizationObjectivePtr &a, double w);
    }
}

#endif
