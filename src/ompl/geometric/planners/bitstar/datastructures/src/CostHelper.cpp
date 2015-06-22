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

// My definition:
#include "ompl/geometric/planners/bitstar/datastructures/CostHelper.h"

namespace ompl
{
    namespace geometric
    {
        /////////////////////////////////////////////////////////////////////////////////////////////
        // Public functions:
        BITstar::CostHelper::CostHelper(ompl::base::OptimizationObjectivePtr opt,
                                        std::shared_ptr<const VertexPtrList> startVertices,
                                        std::shared_ptr<const VertexPtrList> goalVertices)
          : opt_(std::move(opt)), startVerticesPtr_(std::move(startVertices)), goalVerticesPtr_(std::move(goalVertices))
        {
        }

        //////////////////
        // Heuristic helper functions
        ompl::base::Cost BITstar::CostHelper::lowerBoundHeuristicVertex(const VertexConstPtr &vertex) const
        {
            return this->combineCosts(this->costToComeHeuristic(vertex), this->costToGoHeuristic(vertex));
        }

        ompl::base::Cost BITstar::CostHelper::currentHeuristicVertex(const VertexConstPtr &vertex) const
        {
            return this->combineCosts(vertex->getCost(), this->costToGoHeuristic(vertex));
        }

        ompl::base::Cost BITstar::CostHelper::lowerBoundHeuristicEdge(const VertexConstPtrPair &edgePair) const
        {
            return this->combineCosts(this->lowerBoundHeuristicTarget(edgePair),
                                      this->costToGoHeuristic(edgePair.second));
        }

        ompl::base::Cost BITstar::CostHelper::currentHeuristicEdge(const VertexConstPtrPair &edgePair) const
        {
            return this->combineCosts(this->currentHeuristicTarget(edgePair), this->costToGoHeuristic(edgePair.second));
        }

        ompl::base::Cost BITstar::CostHelper::lowerBoundHeuristicTarget(const VertexConstPtrPair &edgePair) const
        {
            return this->combineCosts(this->costToComeHeuristic(edgePair.first), this->edgeCostHeuristic(edgePair));
        }

        ompl::base::Cost BITstar::CostHelper::currentHeuristicTarget(const VertexConstPtrPair &edgePair) const
        {
            return this->combineCosts(edgePair.first->getCost(), this->edgeCostHeuristic(edgePair));
        }

        ompl::base::Cost BITstar::CostHelper::costToComeHeuristic(const VertexConstPtr &vertex) const
        {
            // Variable
            // The current best cost to the state, initialize to infinity
            ompl::base::Cost curBest = this->infiniteCost();

            // Iterate over the list of starts, finding the minimum estimated cost-to-come to the state
            for (const auto &startVertex : *startVerticesPtr_)
            {
                // Update the cost-to-come as the better of the best so far and the new one
                curBest = this->betterCost(curBest,
                                           this->motionCostHeuristic(startVertex->stateConst(), vertex->stateConst()));
            }

            // Return
            return curBest;
        }

        ompl::base::Cost BITstar::CostHelper::edgeCostHeuristic(const VertexConstPtrPair &edgePair) const
        {
            return this->motionCostHeuristic(edgePair.first->stateConst(), edgePair.second->stateConst());
        }

        ompl::base::Cost BITstar::CostHelper::costToGoHeuristic(const VertexConstPtr &vertex) const
        {
            // Variable
            // The current best cost to a goal from the state, initialize to infinity
            ompl::base::Cost curBest = this->infiniteCost();

            // Iterate over the list of goals, finding the minimum estimated cost-to-go from the state
            for (const auto &goalVertex : *goalVerticesPtr_)
            {
                // Update the cost-to-go as the better of the best so far and the new one
                curBest = this->betterCost(curBest,
                                           this->motionCostHeuristic(vertex->stateConst(), goalVertex->stateConst()));
            }

            // Return
            return curBest;
        }
        //////////////////

        //////////////////
        // Cost helper functions
        bool BITstar::CostHelper::isCostWorseThan(const ompl::base::Cost &a, const ompl::base::Cost &b) const
        {
            // If b is better than a, then a is worse than b
            return this->isCostBetterThan(b, a);
        }

        bool BITstar::CostHelper::isCostNotEquivalentTo(const ompl::base::Cost &a, const ompl::base::Cost &b) const
        {
            // If a is better than b, or b is better than a, then they are not equal
            return this->isCostBetterThan(a, b) || this->isCostBetterThan(b, a);
        }

        bool BITstar::CostHelper::isCostBetterThanOrEquivalentTo(const ompl::base::Cost &a,
                                                                 const ompl::base::Cost &b) const
        {
            // If b is not better than a, then a is better than, or equal to, b
            return !this->isCostBetterThan(b, a);
        }

        bool BITstar::CostHelper::isCostWorseThanOrEquivalentTo(const ompl::base::Cost &a,
                                                                const ompl::base::Cost &b) const
        {
            // If a is not better than b, than a is worse than, or equal to, b
            return !this->isCostBetterThan(a, b);
        }

        ompl::base::Cost BITstar::CostHelper::combineCosts(const ompl::base::Cost &a, const ompl::base::Cost &b,
                                                           const ompl::base::Cost &c) const
        {
            return this->combineCosts(a, this->combineCosts(b, c));
        }

        ompl::base::Cost BITstar::CostHelper::combineCosts(const ompl::base::Cost &a, const ompl::base::Cost &b,
                                                           const ompl::base::Cost &c, const ompl::base::Cost &d) const
        {
            return this->combineCosts(a, this->combineCosts(b, c, d));
        }

        double BITstar::CostHelper::fractionalChange(const ompl::base::Cost &newCost,
                                                     const ompl::base::Cost &oldCost) const
        {
            return this->fractionalChange(newCost, oldCost, oldCost);
        }

        double BITstar::CostHelper::fractionalChange(const ompl::base::Cost &newCost, const ompl::base::Cost &oldCost,
                                                     const ompl::base::Cost &refCost) const
        {
            // If the old cost is not finite, than we call that infinite percent improvement
            if (this->isFinite(oldCost) == false)
            {
                // Return infinity (but not beyond)
                return std::numeric_limits<double>::infinity();
            }
            else
            {
                // Calculate and return
                return (newCost.value() - oldCost.value()) / refCost.value();
            }
        }
    }  // geometric
}  // ompl
