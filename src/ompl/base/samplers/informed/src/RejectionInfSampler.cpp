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
        RejectionInfSampler::RejectionInfSampler(const ProblemDefinitionPtr &probDefn, unsigned int maxNumberCalls)
          : InformedSampler(probDefn, maxNumberCalls)
        {
            // Create the basic sampler
            baseSampler_ = InformedSampler::space_->allocDefaultStateSampler();

            // Warn if a cost-to-go heuristic is not defined
            if (!InformedSampler::opt_->hasCostToGoHeuristic())
            {
                OMPL_WARN("RejectionInfSampler: The optimization objective does not have a cost-to-go heuristic "
                          "defined. Informed sampling will likely have little to no effect.");
            }
            // No else
        }

        bool RejectionInfSampler::sampleUniform(State *statePtr, const Cost &maxCost)
        {
            // Variable
            // The persistent iteration counter:
            unsigned int iter = 0u;

            // Call the sampleUniform helper function with my iteration counter:
            return sampleUniform(statePtr, maxCost, &iter);
        }

        bool RejectionInfSampler::sampleUniform(State *statePtr, const Cost &minCost, const Cost &maxCost)
        {
            // Variable
            // Whether we were successful in creating an informed sample. Initially not:
            bool foundSample = false;

            // Spend numIters_ iterations trying to find an informed sample:
            for (unsigned int i = 0u; i < InformedSampler::numIters_ && !foundSample; ++i)
            {
                // Call the helper function for the larger cost. It will move our iteration counter:
                foundSample = sampleUniform(statePtr, maxCost, &i);

                // Did we find a sample?
                if (foundSample)
                {
                    // We did, but it only satisfied the upper bound. Check that it meets the lower bound.

                    // Variables
                    // The cost of the sample we found:
                    Cost sampledCost = InformedSampler::heuristicSolnCost(statePtr);

                    // Check if the sample's cost is greater than or equal to the lower bound
                    foundSample = InformedSampler::opt_->isCostEquivalentTo(minCost, sampledCost) ||
                                  InformedSampler::opt_->isCostBetterThan(minCost, sampledCost);
                }
                // No else, no sample was found.
            }

            // One way or the other, we're done:
            return foundSample;
        }

        bool RejectionInfSampler::hasInformedMeasure() const
        {
            return false;
        }

        double RejectionInfSampler::getInformedMeasure(const Cost & /*currentCost*/) const
        {
            return InformedSampler::space_->getMeasure();
        }

        double RejectionInfSampler::getInformedMeasure(const Cost & /*minCost*/, const Cost & /*maxCost*/) const
        {
            return InformedSampler::space_->getMeasure();
        }

        bool RejectionInfSampler::sampleUniform(State *statePtr, const Cost &maxCost, unsigned int *iterPtr)
        {
            // Variable
            // Whether we were successful in creating an informed sample. Initially not:
            bool foundSample = false;

            // Make numIters_ attempts at finding a sample whose heuristic estimate of solution cost through the sample
            // is better than maxCost by sampling the entire planning domain
            for (/* Provided iteration counter */; *iterPtr < InformedSampler::numIters_ && !foundSample;
                 ++(*iterPtr))
            {
                // Get a sample:
                baseSampler_->sampleUniform(statePtr);

                // Check if it's found, i.e., if f(state) <= maxCost
                foundSample =
                    InformedSampler::opt_->isCostBetterThan(InformedSampler::heuristicSolnCost(statePtr), maxCost);
            }

            // All done, one way or the other:
            return foundSample;
        }
    };  // base
};      // ompl
