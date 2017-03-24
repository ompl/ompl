/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Rice University
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
*   * Neither the name of the Rice University nor the names of its
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

/* Author: Javier V. Gómez */

#ifndef OMPL_BASE_SAMPLERS_CFOREST_STATE_SAMPLER_
#define OMPL_BASE_SAMPLERS_CFOREST_STATE_SAMPLER_

#include "ompl/base/StateSpace.h"

#include <mutex>
#include <utility>

namespace ompl
{
    namespace base
    {
        /** \brief Extended state sampler to use with the CForest planning algorithm. It wraps the user-specified
            state sampler.*/
        class CForestStateSampler : public StateSampler
        {
        public:
            /** \brief Constructor */
            CForestStateSampler(const StateSpace *space, StateSamplerPtr sampler)
              : StateSampler(space), sampler_(std::move(sampler))
            {
            }

            /** \brief Destructor */
            ~CForestStateSampler() override
            {
                clear();
            }

            /** \brief It will sample the next state of the vector StatesToSample_. If this is empty,
                it will call the sampleUniform() method of the specified sampler. */
            void sampleUniform(State *state) override;

            /** \brief It will sample the next state of the vector StatesToSample_. If this is empty,
                it will call the sampleUniformNear() method of the specified sampler. */
            void sampleUniformNear(State *state, const State *near, double distance) override;

            /** \brief It will sample the next state of the vector StatesToSample_. If this is empty,
                it will call the sampleGaussian() method of the specified sampler. */
            void sampleGaussian(State *state, const State *mean, double stdDev) override;

            const StateSpace *getStateSpace() const
            {
                return space_;
            }

            /** \brief Fills the vector StatesToSample_ of states to be sampled in the next
                calls to sampleUniform(), sampleUniformNear() or sampleGaussian(). */
            void setStatesToSample(const std::vector<const State *> &states);

            void clear();

        protected:
            /** \brief Extracts the next sample when statesToSample_ is not empty. */
            void getNextSample(State *state);

            /** \brief States to be sampled */
            std::vector<State *> statesToSample_;

            /** \brief Underlying, user-specified state sampler. */
            StateSamplerPtr sampler_;

            /** \brief Lock to control the access to the statesToSample_ vector. */
            std::mutex statesLock_;
        };
    }
}

#endif
