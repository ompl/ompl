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

#include "ompl/base/samplers/informed/RejectionInfSampler.h"
#include "ompl/base/OptimizationObjective.h"

namespace ompl
{
    namespace base
    {
        // The default rejection-sampling class:
        RejectionInfSampler::RejectionInfSampler(const StateSpace* space, const ProblemDefinitionPtr probDefn, const Cost* bestCost)
          : InformedStateSampler(space, probDefn, bestCost)
        {
            // Create the basic sampler
            baseSampler_ = StateSampler::space_->allocDefaultStateSampler();

            // Warn if a cost-to-go heuristic is not defined
            if (InformedStateSampler::opt_->hasCostToGoHeuristic() == false)
            {
                OMPL_WARN("RejectionInfSampler: The optimization objective does not have a cost-to-go heuristic defined. Informed sampling will likely have little to no effect.");
            }
            // No else
        }

        void RejectionInfSampler::sampleUniform(State* statePtr, const Cost& maxCost)
        {
            // Sample from the entire domain as long as the heuristic estimate of solution cost through the sample is worse than maxCost.
            // i.e., stop when f(state) <= maxCost
            do
            {
                baseSampler_->sampleUniform(statePtr);
            }
            while (this->isCostWorseThan(InformedStateSampler::heuristicSolnCost(statePtr), maxCost));
        }

        void RejectionInfSampler::sampleUniform(State* statePtr, const Cost& minCost, const Cost& maxCost)
        {
            // Sample from the larger cost bound as long as the heuristic estimate of the solution cost through sample is better than the smaller bound.
            // i.e., stop when minCost <= f(state) <= maxCost
            do
            {
                this->sampleUniform(statePtr, maxCost);
            }
            while (InformedStateSampler::opt_->isCostBetterThan(InformedStateSampler::heuristicSolnCost(statePtr), minCost));
        }

        bool RejectionInfSampler::hasInformedMeasure() const
        {
            return false;
        }

        double RejectionInfSampler::getInformedMeasure() const
        {
            return StateSampler::space_->getMeasure();
        }

        double RejectionInfSampler::getInformedMeasure(const Cost& /*currentCost*/) const
        {
            return StateSampler::space_->getMeasure();
        }

        double RejectionInfSampler::getInformedMeasure(const Cost& /*minCost*/, const Cost& /*maxCost*/) const
        {
            return StateSampler::space_->getMeasure();
        }

        bool RejectionInfSampler::isCostWorseThan(const Cost& c1, const Cost& c2) const
        {
            // If c2 is better than c1, then c1 is worse than c2
            return InformedStateSampler::opt_->isCostBetterThan(c2, c1);
        }
    }; // base
};  // ompl
