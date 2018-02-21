/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, Rice University
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

/* Author: Bryce Willey */

#ifndef OMPL_BASE_SAMPLERS_GRADIENT_VALID_STATE_SAMPLER_
#define OMPL_BASE_SAMPLERS_GRADIENT_VALID_STATE_SAMPLER_

#include "ompl/base/StateSampler.h"

// We need to be able to sample with a cost.
#include "ompl/base/Cost.h"

// For std::function
#include <functional>

// For the collision info struct and the existing definition of a Jacobian Function.
#include "ompl/base/objectives/CollisionEvaluator.h"

#include <fstream>
#include <iostream>

namespace ompl
{
    namespace base
    {
        OMPL_CLASS_FORWARD(GradientValidStateSampler);

        /** \brief A state sampler that uses gradient descent to sample along
         *         the medial axis of a cost.
         */
        class GradientValidStateSampler : public ValidStateSampler
        {
        public:
            GradientValidStateSampler(const SpaceInformation *si, double epsilon=0.1);
            ~GradientValidStateSampler() override = default;

            bool sample(State *state) override;
        
            virtual bool sampleWithEpsilon(State *state, double epsilon)=0;

            /**GradientValidStateSampler.1
             * TODO(brycew): why does this need to be implemented? Figure out a way around.
             */
            bool sampleNear(State *state, const State *near, double distance) override;

        protected:
            double epsilon_ = 0.1;
            StateSamplerPtr sampler_;
            unsigned int dof_;
            std::ofstream of_;
        };
    }
}

#endif