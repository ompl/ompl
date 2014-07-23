/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
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

//#include "ompl/base/StateSampler.h"
#include "ompl/base/StateSpace.h"


// TODO: This is kind of higher-level sampler. The actual sampler to use
// have to be chosen. Now it is just a version of UniformValidSS.
namespace ompl
{
    namespace base
    {

        /** \brief State sampler for the R<sup>n</sup> state space */
        class CForestStateSampler : public StateSampler
        {
        public:

            /** \brief Constructor */
            CForestStateSampler(const StateSpace *space) : StateSampler(space)
            {
            }

            virtual void sampleUniform(State *state);
            /** \brief Sample a state such that each component state[i] is
                uniformly sampled from [near[i]-distance, near[i]+distance].
                If this interval exceeds the state space bounds, the
                interval is truncated. */
            virtual void sampleUniformNear(State *state, const State *near, const double distance);
            /** \brief Sample a state such that each component state[i] has
                a Gaussian distribution with mean mean[i] and standard
                deviation stdDev. If the sampled value exceeds the state
                space boundary, it is thresholded to the nearest boundary. */
            virtual void sampleGaussian(State *state, const State *mean, const double stdDev);

            void addStateToSample(const State *state);

            void setStatesToSample(const std::vector<State *> &states);

            void getNextSample(State *state);

        protected:

            std::vector<State *> statesToSample_;
        };

    }
}

#endif
