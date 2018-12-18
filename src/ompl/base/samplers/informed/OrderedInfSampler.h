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

#ifndef OMPL_BASE_SAMPLERS_INFORMED_ORDERED_INFORMED_SAMPLER_
#define OMPL_BASE_SAMPLERS_INFORMED_ORDERED_INFORMED_SAMPLER_

// We inherit from InformedStateSampler
#include "ompl/base/samplers/InformedStateSampler.h"

// For a priority queue
#include <queue>
// For std::function
#include <functional>

namespace ompl
{
    namespace base
    {
        /** \brief An informed sampler wrapper that generates \e m samples and then returns them in order of the
           heuristic.

        */
        class OrderedInfSampler : public InformedSampler
        {
        public:
            /** \brief Construct an ordering wrapper around the provided informed sampler. */
            OrderedInfSampler(const InformedSamplerPtr &infSamplerPtr, unsigned int batchSize);
            ~OrderedInfSampler() override = default;

            /** \brief Sample uniformly in the subset of the state space whose heuristic solution estimates are less
             * than the provided cost, i.e. in the interval [0, maxCost). Returns false if such a state was not found in
             * the specified number of iterations. */
            bool sampleUniform(State *statePtr, const Cost &maxCost) override;

            /** \brief Sample uniformly in the subset of the state space whose heuristic solution estimates are between
             * the provided costs, [minCost, maxCost). Returns false if such a state was not found in the specified
             * number of iterations. */
            bool sampleUniform(State *statePtr, const Cost &minCost, const Cost &maxCost) override;

            /** \brief Whether the wrapped sampler can provide a measure of the informed subset */
            bool hasInformedMeasure() const override;

            /** \brief The measure of the subset of the state space defined by the current solution cost that is being
             * searched. Passes through to the wrapper sampler. */
            double getInformedMeasure(const Cost &currentCost) const override;

            /** Set the seeds of the underlying RNGs */
            void setLocalSeed(std::uint_fast32_t localSeed) override
            {
                infSampler_->setLocalSeed(localSeed);
            };

        private:
            // Variables
            /** \brief The informed sampler to use. */
            InformedSamplerPtr infSampler_;
            /** \brief The batch size to use. */
            unsigned int batchSize_;
            /** \brief The container of ordered samples. */
            std::priority_queue<State *, std::vector<State *>, std::function<bool(const State *, const State *)>>
                orderedSamples_;

            // Functions
            /** \brief The ordering function for the priority queue */
            bool queueComparator(const State *a, const State *b);

            /** \brief Construct a batch of samples to return with costs in the interval [0, maxCost) */
            void createBatch(const Cost &maxCost);

            /** \brief Construct a batch of samples to return with costs in the interval [minCost, maxCost) */
            void createBatch(const Cost &minCost, const Cost &maxCost);

            /** \brief Clear a batch of prepared samples */
            void clearBatch();
        };
    }
}

#endif  // OMPL_BASE_SAMPLERS_INFORMED_REJECTION_INFORMED_SAMPLER_
