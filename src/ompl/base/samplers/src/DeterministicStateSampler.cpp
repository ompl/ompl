/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Robert Bosch GmbH
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
 *   * Neither the name of the Robert Bosch GmbH nor the names of its
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

/* Author: Leonard Bruns */

#include "ompl/base/samplers/DeterministicStateSampler.h"
#include "ompl/base/samplers/deterministic/HaltonSequence.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/base/spaces/SO2StateSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"

#include <iostream>
#include <boost/math/constants/constants.hpp>

namespace ompl
{
    namespace base
    {
        DeterministicStateSampler::DeterministicStateSampler(const StateSpace *space, DeterministicSamplerType type)
          : StateSampler(space)
        {
            switch (type)
            {
                case HALTON:
                    sequence_ptr_ = std::make_shared<HaltonSequence>(space->getDimension());
                    break;
                default:
                    OMPL_WARN("Unknown deterministic sampler type specified, using Halton instead.");

                    break;
            }
        }

        DeterministicStateSampler::DeterministicStateSampler(const StateSpace *space,
                                                             std::shared_ptr<DeterministicSequence> sequence_ptr)
          : StateSampler(space), sequence_ptr_(sequence_ptr)
        {
        }

        void SO2DeterministicStateSampler::sampleUniform(State *state)
        {
            auto sample = sequence_ptr_->sample();
            state->as<SO2StateSpace::StateType>()->value =
                -boost::math::constants::pi<double>() + sample[0] * 2 * boost::math::constants::pi<double>();
        }

        void SO2DeterministicStateSampler::sampleUniformNear(State *, const State *, double)
        {
            OMPL_WARN("Deterministic sampler does not support near sampling.");
        }

        void SO2DeterministicStateSampler::sampleGaussian(State *, const State *, double)
        {
            OMPL_WARN("Deterministic sampler does not support Gaussian sampling.");
        }

        void RealVectorDeterministicStateSampler::sampleUniform(State *state)
        {
            auto sample = sequence_ptr_->sample();

            const unsigned int dim = space_->getDimension();

            const RealVectorBounds &bounds = static_cast<const RealVectorStateSpace *>(space_)->getBounds();

            if (stretch_)
            {
                auto *rstate = static_cast<RealVectorStateSpace::StateType *>(state);
                for (unsigned int i = 0; i < dim; ++i)
                    rstate->values[i] = bounds.low[i] + sample[i] * (bounds.high[i] - bounds.low[i]);
            }
            else
            {
                auto *rstate = static_cast<RealVectorStateSpace::StateType *>(state);
                for (unsigned int i = 0; i < dim; ++i)
                    rstate->values[i] = sample[i];
            }
        }

        void RealVectorDeterministicStateSampler::sampleUniformNear(State *, const State *, double)
        {
            OMPL_WARN("Deterministic sampler does not support near sampling.");
        }

        void RealVectorDeterministicStateSampler::sampleGaussian(State *, const State *, double)
        {
            OMPL_WARN("Deterministic sampler does not support Gaussian sampling.");
        }

        void SE2DeterministicStateSampler::sampleUniform(State *state)
        {
            auto sample = sequence_ptr_->sample();

            const RealVectorBounds &bounds = static_cast<const SE2StateSpace *>(space_)->getBounds();

            auto se2_state_ptr = static_cast<SE2StateSpace::StateType *>(state);
            if (stretch_rv_)
            {
                se2_state_ptr->setX(bounds.low[0] + sample[0] * (bounds.high[0] - bounds.low[0]));
                se2_state_ptr->setY(bounds.low[1] + sample[1] * (bounds.high[1] - bounds.low[1]));
            }
            else
                se2_state_ptr->setXY(sample[0], sample[1]);

            if (stretch_so2_)
                se2_state_ptr->setYaw(-boost::math::constants::pi<double>() +
                                      sample[2] * 2 * boost::math::constants::pi<double>());
            else
                se2_state_ptr->setYaw(sample[2]);
        }

        void SE2DeterministicStateSampler::sampleUniformNear(State *, const State *, double)
        {
            OMPL_WARN("Deterministic sampler does not support near sampling.");
        }

        void SE2DeterministicStateSampler::sampleGaussian(State *, const State *, double)
        {
            OMPL_WARN("Deterministic sampler does not support Gaussian sampling.");
        }
    }  // namespace base
}  // namespace ompl
