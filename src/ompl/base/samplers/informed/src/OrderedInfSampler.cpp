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

#include "ompl/base/samplers/informed/OrderedInfSampler.h"
#include "ompl/base/OptimizationObjective.h"

namespace ompl
{
    namespace base
    {
        // The default rejection-sampling class:
        OrderedInfSampler::OrderedInfSampler(const InformedSamplerPtr &infSamplerPtr, unsigned int batchSize)
          : InformedSampler(infSamplerPtr->getProblemDefn(), infSamplerPtr->getMaxNumberOfIters())
          , infSampler_(infSamplerPtr)
          , batchSize_(batchSize)
          , orderedSamples_([this](const State *lhs, const State *rhs)
                            {
                                return queueComparator(lhs, rhs);
                            })
        {
        }

        bool OrderedInfSampler::sampleUniform(State *statePtr, const Cost &maxCost)
        {
            // Variables
            // Whether a sampler has been found and returned
            bool found = false;

            // Repeat until a valid pointer is found
            while (!found)
            {
                // Check if the batch is empty
                if (orderedSamples_.empty())
                {
                    // It is, recreate:
                    createBatch(maxCost);
                }

                // Does the front of the priority queue meet our requirement (as the requirement may have changed since
                // the batch was generated)
                if (InformedSampler::opt_->isCostBetterThan(InformedSampler::heuristicSolnCost(orderedSamples_.top()),
                                                            maxCost))
                {
                    // Copy the front of the priority queue.
                    InformedSampler::space_->copyState(statePtr, orderedSamples_.top());

                    // Free the pointer in the queue
                    InformedSampler::space_->freeState(orderedSamples_.top());

                    // Pop it
                    orderedSamples_.pop();

                    // And mark that we've found a sample
                    found = true;
                }
                else
                {
                    // It does not, clear the queue
                    clearBatch();
                }
            }

            return found;
        }

        bool OrderedInfSampler::sampleUniform(State *, const Cost &, const Cost &)
        {
            throw ompl::Exception("Not implemented");

            return false;
        }

        bool OrderedInfSampler::hasInformedMeasure() const
        {
            return infSampler_->hasInformedMeasure();
        }

        double OrderedInfSampler::getInformedMeasure(const Cost &currentCost) const
        {
            return infSampler_->getInformedMeasure(currentCost);
        }

        bool OrderedInfSampler::queueComparator(const State *a, const State *b)
        {
            return InformedSampler::opt_->isCostBetterThan(InformedSampler::heuristicSolnCost(b),
                                                           InformedSampler::heuristicSolnCost(a));
        }

        void OrderedInfSampler::createBatch(const Cost &maxCost)
        {
            // Allocate, create and store batchSize_ samples
            for (unsigned int i = 0u; i < batchSize_; ++i)
            {
                // Allocate a state pointer
                State *newStatePtr = InformedSampler::space_->allocState();

                // Sample the state pointer using the wrapped sampler
                infSampler_->sampleUniform(newStatePtr, maxCost);

                // Store it into the queue
                orderedSamples_.push(newStatePtr);
            }
        }

        void OrderedInfSampler::createBatch(const Cost &, const Cost &)
        {
            throw ompl::Exception("Not implemented");
        }

        void OrderedInfSampler::clearBatch()
        {
            // Iterate through the entire queue, removing the element and freeing it.
            while (!orderedSamples_.empty())
            {
                // Free the front state
                InformedSampler::space_->freeState(orderedSamples_.top());

                // Pop the front state
                orderedSamples_.pop();
            }
        }
    };  // base
};      // ompl
