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

#include "ompl/base/SpaceInformation.h"
#include "ompl/util/ClassForward.h"
#include <boost/noncopyable.hpp>
#include <boost/concept_check.hpp>

namespace ompl
{
    namespace base
    {   
        struct Cost
        {
            explicit Cost(double v = 0.0) : v(v) {}
            double v;
        };

        std::ostream& operator<<(std::ostream& stream, Cost c);

        class Goal;

        /** \brief The definition of a function which returns an admissible estimate of the optimal path cost from a given state to a goal. */
        typedef boost::function<Cost (const State*, const Goal*)> CostToGoHeuristic;
      
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::OptimizationObjective */
        OMPL_CLASS_FORWARD(OptimizationObjective);
        /// @endcond

        /// @cond IGNORE
        OMPL_CLASS_FORWARD(Path);
        /// @endcond	

        /** \class ompl::base::OptimizationObjectivePtr
            \brief A boost shared pointer wrapper for ompl::base::OptimizationObjective */

        /** \brief Abstract definition of optimization objectives.

            \note This implementation has greatly benefited from discussions with <a href="http://www.cs.indiana.edu/~hauserk/">Kris Hauser</a> */
        class OptimizationObjective : private boost::noncopyable
        {
        public:
            /** \brief Constructor. The objective must always know the space information it is part of. The cost threshold for objective satisfaction defaults to 0.0. */
            OptimizationObjective(const SpaceInformationPtr &si);

            virtual ~OptimizationObjective(void)
            {
            }

            /** \brief Get the description of this optimization objective */
            const std::string& getDescription(void) const;

            /** \brief Verify that our objective is satisfied already and we can stop planning */
            virtual bool isSatisfied(Cost c) const;

            /** \brief Returns the cost threshold currently being checked for objective satisfaction */
            Cost getCostThreshold(void) const;

            /** \brief Set the cost threshold for objective satisfaction. When a path is found with a cost better than the cost threshold, the objective is considered satisfied. */
            void setCostThreshold(Cost c);

            /** \brief Get the cost that corresponds to an entire path. This implementation assumes \e Path is of type \e PathGeometric.*/
            virtual Cost getCost(const Path &path) const;

	    /** \brief Check whether the the cost \e c1 is considered better than the cost \e c2. By default, this returns true only if c1 is less by at least some threshold amount, for numerical robustness. */
	    virtual bool isCostBetterThan(Cost c1, Cost c2) const;
	 
	    /** \brief Evaluate a cost map defined on the state space at a state \e s. Default implementation maps all states to 1.0. */
	    virtual Cost stateCost(const State *s) const;

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
	    virtual bool isSymmetric(void) const;

	    /** \brief Compute the average state cost of this objective by taking a sample of \e numStates states */
	    virtual Cost averageStateCost(unsigned int numStates) const;

            /** \brief Set the cost-to-go heuristic function for this objective. The cost-to-go heuristic is a function which returns an admissible estimate of the optimal path cost from a given state to a goal, where "admissible" means that the estimated cost is always less than the true optimal cost. */
            void setCostToGoHeuristic(const CostToGoHeuristic& costToGo);

            /** Uses a cost-to-go heuristic to calculate an admissible estimate of the optimal cost from a given state to a given goal. If no cost-to-go heuristic has been specified with setCostToGoHeuristic(), this function just returns the identity cost, which is sure to be an admissible heuristic if there are no negative costs. */
            Cost costToGo(const State* state, const Goal* goal) const;

            /** Defines an admissible estimate on the optimal cost on the motion between states \e s1 and \e s2. An admissible estimate always undervalues the true optimal cost of the motion. Used by some planners to speed up planning. The default implementation of this method returns this objective's identity cost, which is sure to be an admissible heuristic if there are no negative costs. */
            virtual Cost motionCostHeuristic(const State* s1, const State* s2) const;

            // Returns this objective's SpaceInformation. Needed for operators in MultiOptimizationObjective
            const SpaceInformationPtr& getSpaceInformation(void) const;

        protected:
            /** \brief The space information for this objective */
            SpaceInformationPtr si_;

            /** \brief The description of this optimization objective */
            std::string description_;

            /** \brief The cost threshold used for checking whether this objective has been satisfied during planning */
            Cost threshold_;

            /** \brief The function used for returning admissible estimates on the optimal cost of the path between a given state and goal */
            CostToGoHeuristic costToGoFn_;
        };

        // For use when goal region's distanceGoal() is equivalent to
        // the cost-to-go of a state under the optimization
        // objective. This function assumes that all states within the
        // goal region's threshold have a cost-to-go of exactly zero.
        inline Cost goalRegionCostToGo(const State* state, const Goal* goal);

        /** \brief An optimization objective which corresponds to optimizing path length. */
        class PathLengthOptimizationObjective : public OptimizationObjective
        {
        public:
            PathLengthOptimizationObjective(const SpaceInformationPtr &si);

            /** \brief Motion cost for this objective is defined as
                 the configuration space distance between \e s1 and \e
                 s2, using the method SpaceInformation::distance(). */
            virtual Cost motionCost(const State *s1, const State *s2) const;

            /** \brief the motion cost heuristic for this objective is
                simply the configuration space distance between \e s1
                and \e s2, since this is the optimal cost between any
                two states assuming no obstacles. */
            virtual Cost motionCostHeuristic(const State *s1, const State *s2) const;
        };

        /** \brief Defines optimization objectives where path cost can
            be represented as a path integral over a cost function
            defined over the state space. This cost function is
            specified by implementing the stateCost() method. */
	class StateCostIntegralObjective : public OptimizationObjective
	{
	public:	
            /** \brief If enableMotionCostInterpolation is set to
                true, then calls to motionCost() will divide the
                motion segment into smaller parts (the number of parts
                being defined by StateSpace::validSegmentCount()) for
                more accurate cost integral computation (but this
                takes more computation time). If
                enableMotionCostInterpolation is false (the default),
                only the two endpoint states are used for motion cost
                computation.
             */
	    StateCostIntegralObjective(const SpaceInformationPtr &si, 
                                       bool enableMotionCostInterpolation = false);

	    /** \brief Compute the cost of a path segment from \e s1 to \e s2 (including endpoints)
		\param s1 start state of the motion to be evaluated
		\param s2 final state of the motion to be evaluated
		\param cost the cost of the motion segment

		By default, this function computes
		\f{eqnarray*}{
		\mbox{cost} &=& \frac{cost(s_1) + cost(s_2)}{2}\vert s_1 - s_2 \vert
		\f}
                
                If enableMotionCostInterpolation was specified as true
                in constructing this object, the cost will be computed
                by separating the motion into
                StateSpace::validSegmentCount() segments, using the
                above formula to compute the cost of each of those
                segments, and adding them up.
	    */
	    virtual Cost motionCost(const State *s1, const State *s2) const;

            /** \brief Returns whether this objective subdivides
                motions into smaller segments for more accurate motion
                cost computation. Motion cost interpolation is
                disabled by default.
             */
            bool isMotionCostInterpolationEnabled(void) const;
        protected:

            // If true, then motionCost() will more accurately compute
            // the cost of a motion by taking small steps along the
            // motion and accumulating the cost. This sacrifices speed
            // for accuracy. If false, the motion cost will be
            // approximated by taking the average of the costs at the
            // two end points, and normalizing by the distance between
            // the two end points.
            bool interpolateMotionCost_;

            /** \brief Helper method which uses the trapezoidal rule
             to approximate the integral of the cost between two
             states of distance \e distance and costs \e c1 and \e
             c2 **/
            Cost trapezoid(Cost c1, Cost c2, double distance) const;
	};

	class MechanicalWorkOptimizationObjective : public OptimizationObjective
	{
	public:
	    MechanicalWorkOptimizationObjective(const SpaceInformationPtr &si,
                                                double pathLengthWeight = 0.00001);

	    virtual double getPathLengthWeight(void) const;
	    virtual void setPathLengthWeight(double weight);

	    virtual Cost motionCost(const State *s1, const State *s2) const;

	protected:
	    double pathLengthWeight_;
	};

        // The cost of a path is defined as the worst state cost over
        // the entire path. This objective attempts to find the path
        // with the "best worst cost" over all paths.
        class MinimaxObjective : public OptimizationObjective
        {
        public:
            MinimaxObjective(const SpaceInformationPtr &si);

            // assumes all costs are worse than identity
            virtual Cost motionCost(const State *s1, const State *s2) const;
            virtual Cost combineCosts(Cost c1, Cost c2) const;
        };

        class MaximizeMinClearanceObjective : public MinimaxObjective
        {
        public:
            MaximizeMinClearanceObjective(const SpaceInformationPtr &si);

            virtual Cost stateCost(const State* s) const;
            virtual bool isCostBetterThan(Cost c1, Cost c2) const;
            virtual Cost identityCost(void) const;
            virtual Cost infiniteCost(void) const;
        };

        class MultiOptimizationObjective : public OptimizationObjective
        {
        public:
            MultiOptimizationObjective(const SpaceInformationPtr &si);

            void addObjective(const OptimizationObjectivePtr& objective, 
                              double weight);

            std::size_t getObjectiveCount(void) const;

            const OptimizationObjectivePtr& getObjective(unsigned int idx) const;

            double getObjectiveWeight(unsigned int idx) const;
            void setObjectiveWeight(unsigned int idx, double weight);

            void lock(void);
            bool isLocked(void) const;

            // For now, assumes we simply use addition to add up all
            // objective cost values, where each individual value is
            // scaled by its weight
            virtual Cost stateCost(const State* s) const;

            // Same as stateCost
            virtual Cost motionCost(const State* s1, const State* s2) const;

        protected:

            struct Component
            {
                Component(const OptimizationObjectivePtr& obj, double weight);
                OptimizationObjectivePtr objective;
                double weight;
            };

            std::vector<Component> components_;
            bool locked_;

            friend OptimizationObjectivePtr operator+(const OptimizationObjectivePtr &a,
                                                      const OptimizationObjectivePtr &b);

            friend OptimizationObjectivePtr operator*(double w, const OptimizationObjectivePtr &a);

            friend OptimizationObjectivePtr operator*(const OptimizationObjectivePtr &a, double w);
        };

        OptimizationObjectivePtr operator+(const OptimizationObjectivePtr &a,
                                           const OptimizationObjectivePtr &b);

        OptimizationObjectivePtr operator*(double w, const OptimizationObjectivePtr &a);
      
        OptimizationObjectivePtr operator*(const OptimizationObjectivePtr &a, double w);
    }
}

#endif
