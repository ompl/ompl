/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, University of Toronto
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
*   * Neither the name of the University of Toronto nor the names of its
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

/* Authors: Jonathan Gammell */

#ifndef OMPL_GEOMETRIC_PLANNERS_BITSTAR_DATASTRUCTURES_COSTHELPER_
#define OMPL_GEOMETRIC_PLANNERS_BITSTAR_DATASTRUCTURES_COSTHELPER_

// OMPL:
// The cost class:
#include "ompl/base/Cost.h"
// The optimization objective class:
#include "ompl/base/OptimizationObjective.h"

// BIT*:
// I am member class of the BITstar class, so I need to include it's definition to be aware of the class BITstar. It has
// a forward declaration to me.
#include "ompl/geometric/planners/bitstar/BITstar.h"
// The vertex class:
#include "ompl/geometric/planners/bitstar/datastructures/Vertex.h"

namespace ompl
{
    namespace geometric
    {
        /** @anchor CostHelper
        \par Short Description
        A class that consolidates all the various heuristic calculations for vertices/edges in a graph into one place.
        Used by \ref gBITstar "BIT*".
        */

        /** \brief A helper class to handle the various heuristic functions in one place. */
        class BITstar::CostHelper
        {
        public:
            ////////////////////////////////
            // Public functions:
            /** \brief Construct the heuristic helper. */
            CostHelper(ompl::base::OptimizationObjectivePtr opt, std::shared_ptr<const VertexPtrList> startVertices,
                       std::shared_ptr<const VertexPtrList> goalVertices);

            virtual ~CostHelper() = default;

            //////////////////
            // Heuristic helper functions
            /** \brief Calculates a heuristic estimate of the cost of a solution constrained to pass through a vertex,
             * independent of the current cost-to-come. I.e., combines the heuristic estimates of the cost-to-come and
             * cost-to-go. */
            ompl::base::Cost lowerBoundHeuristicVertex(const VertexConstPtr &vertex) const;

            /** \brief Calculates a heuristic estimate of the cost of a solution constrained to pass through a vertex,
             * dependent on the current cost-to-come. I.e., combines the current cost-to-come with a heuristic estimate
             * of the cost-to-go. */
            ompl::base::Cost currentHeuristicVertex(const VertexConstPtr &vertex) const;

            /** \brief Calculates a heuristic estimate of the cost of a solution constrained to go through an edge,
             * independent of the cost-to-come of the parent state. I.e., combines the heuristic estimates of the
             * cost-to-come, edge cost, and cost-to-go. */
            ompl::base::Cost lowerBoundHeuristicEdge(const VertexConstPtrPair &edgePair) const;

            /** \brief Calculates a heuristic estimate of the cost of a solution constrained to go through an edge,
             * dependent on the cost-to-come of the parent state. I.e., combines the current cost-to-come with heuristic
             * estimates of the edge cost, and cost-to-go. */
            ompl::base::Cost currentHeuristicEdge(const VertexConstPtrPair &edgePair) const;

            /** \brief Calculates a heuristic estimate of the cost of a path to the \e target of an edge, independent of
             * the current cost-to-come of the parent state. I.e., combines heuristics estimates of the cost-to-come and
             * the edge cost. */
            ompl::base::Cost lowerBoundHeuristicTarget(const VertexConstPtrPair &edgePair) const;

            /** \brief Calculates a heuristic estimate of the cost of a path to the \e target of an edge, dependent on
             * the cost-to-come of the parent state. I.e., combines the current cost-to-come with heuristic estimates of
             * the edge cost. */
            ompl::base::Cost currentHeuristicTarget(const VertexConstPtrPair &edgePair) const;

            /** \brief Calculate a heuristic estimate of the cost-to-come for a Vertex */
            ompl::base::Cost costToComeHeuristic(const VertexConstPtr &vertex) const;

            /** \brief Calculate a heuristic estimate of the cost an edge between two Vertices */
            ompl::base::Cost edgeCostHeuristic(const VertexConstPtrPair &edgePair) const;

            /** \brief Calculate a heuristic estimate of the cost-to-go for a Vertex */
            ompl::base::Cost costToGoHeuristic(const VertexConstPtr &vertex) const;
            //////////////////

            //////////////////
            // Cost helper functions
            /** \brief Compare whether cost a is worse than cost b by checking whether b is better than a. */
            bool isCostWorseThan(const ompl::base::Cost &a, const ompl::base::Cost &b) const;

            /** \brief Compare whether cost a and cost b are not equivalent by checking if either a or b is better than
             * the other. */
            bool isCostNotEquivalentTo(const ompl::base::Cost &a, const ompl::base::Cost &b) const;

            /** \brief Compare whether cost a is better or equivalent to cost b by checking that b is not better than a.
             */
            bool isCostBetterThanOrEquivalentTo(const ompl::base::Cost &a, const ompl::base::Cost &b) const;

            /** \brief Compare whether cost a is worse or equivalent to cost b by checking that a is not better than b.
             */
            bool isCostWorseThanOrEquivalentTo(const ompl::base::Cost &a, const ompl::base::Cost &b) const;

            /** \brief Combine 3 costs */
            ompl::base::Cost combineCosts(const ompl::base::Cost &a, const ompl::base::Cost &b,
                                          const ompl::base::Cost &c) const;

            /** \brief Combine 4 costs */
            ompl::base::Cost combineCosts(const ompl::base::Cost &a, const ompl::base::Cost &b,
                                          const ompl::base::Cost &c, const ompl::base::Cost &d) const;

            /** \brief Calculate the fractional change of cost "newCost" from "oldCost" relative to "oldCost", i.e.,
             * (newCost - oldCost)/oldCost. */
            double fractionalChange(const ompl::base::Cost &newCost, const ompl::base::Cost &oldCost) const;

            /** \brief Calculate the fractional change of cost "newCost" from "oldCost" relative to "refCost", i.e.,
             * (newCost - oldCost)/refCost. */
            double fractionalChange(const ompl::base::Cost &newCost, const ompl::base::Cost &oldCost,
                                    const ompl::base::Cost &refCost) const;
            ////////////////////////////////

            //////////////////
            // Pass-through to OptimizationObjective
            inline bool isFinite(const ompl::base::Cost &a) const
            {
                return opt_->isFinite(a);
            };
            inline bool isCostEquivalentTo(const ompl::base::Cost &a, const ompl::base::Cost &b) const
            {
                return opt_->isCostEquivalentTo(a, b);
            };
            inline bool isCostBetterThan(const ompl::base::Cost &a, const ompl::base::Cost &b) const
            {
                return opt_->isCostBetterThan(a, b);
            };
            inline ompl::base::Cost betterCost(const ompl::base::Cost &a, const ompl::base::Cost &b) const
            {
                return opt_->betterCost(a, b);
            };
            inline ompl::base::Cost combineCosts(const ompl::base::Cost &a, const ompl::base::Cost &b) const
            {
                return opt_->combineCosts(a, b);
            };
            inline ompl::base::Cost infiniteCost() const
            {
                return opt_->infiniteCost();
            };
            inline ompl::base::Cost identityCost() const
            {
                return opt_->identityCost();
            };
            inline ompl::base::Cost motionCostHeuristic(const ompl::base::State *a, const ompl::base::State *b) const
            {
                return opt_->motionCostHeuristic(a, b);
            };
            inline ompl::base::Cost motionCost(const ompl::base::State *a, const ompl::base::State *b) const
            {
                return opt_->motionCost(a, b);
            };

            //////////////////

        private:
            ////////////////////////////////
            // Member variables:
            /** \brief A local pointer to the optimization objective */
            ompl::base::OptimizationObjectivePtr opt_;

            /** \brief A local pointer to the the start states of the problem as vertices. */
            std::shared_ptr<const VertexPtrList> startVerticesPtr_;

            /** \brief A local pointer to the goal states of the problem as vertices. */
            std::shared_ptr<const VertexPtrList> goalVerticesPtr_;
            ////////////////////////////////
        };  // class CostHelper
    }       // geometric
}  // ompl
#endif  // OMPL_GEOMETRIC_PLANNERS_BITSTAR_DATASTRUCTURES_COSTHELPER_
