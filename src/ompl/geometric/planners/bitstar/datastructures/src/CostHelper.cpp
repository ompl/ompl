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

// BIT*:
// The implicit graph:
#include "ompl/geometric/planners/bitstar/datastructures/ImplicitGraph.h"

namespace ompl
{
    namespace geometric
    {
        void BITstar::CostHelper::setup(const ompl::base::OptimizationObjectivePtr &opt, const ImplicitGraphPtr &graph)
        {
            opt_ = opt;
            graphPtr_ = graph;
        }

        /** \brief Clear the CostHelper, returns to state at construction*/
        void BITstar::CostHelper::clear()
        {
            opt_.reset();
            graphPtr_.reset();
        }

        ompl::base::Cost BITstar::CostHelper::costToGoHeuristic(const VertexConstPtr &vertex) const
        {
            // Variable
            // The current best cost to a goal from the state, initialize to infinity
            ompl::base::Cost curBest = this->infiniteCost();

            // Iterate over the vector of goals, finding the minimum estimated cost-to-go from the state
            for (auto goalIter = graphPtr_->goalVerticesBeginConst(); goalIter != graphPtr_->goalVerticesEndConst();
                 ++goalIter)
            {
                // Update the cost-to-go as the better of the best so far and the new one
                curBest = this->betterCost(curBest,
                                           this->motionCostHeuristic(vertex->stateConst(), (*goalIter)->stateConst()));
            }

            // Return
            return curBest;
        }

        ompl::base::Cost BITstar::CostHelper::costToComeHeuristic(const VertexConstPtr &vertex) const
        {
            // Variable
            // The current best cost to the state, initialize to infinity
            ompl::base::Cost curBest = this->infiniteCost();

            // Iterate over the vector of starts, finding the minimum estimated cost-to-come to the state
            for (auto startIter = graphPtr_->startVerticesBeginConst(); startIter != graphPtr_->startVerticesEndConst();
                 ++startIter)
            {
                // Update the cost-to-come as the better of the best so far and the new one
                curBest = this->betterCost(curBest,
                                           this->motionCostHeuristic((*startIter)->stateConst(), vertex->stateConst()));
            }

            // Return
            return curBest;
        }
    }  // geometric
}  // ompl
