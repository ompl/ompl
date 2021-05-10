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

#ifndef OMPL_BASE_DETERMINISTIC_STATE_SAMPLER_
#define OMPL_BASE_DETERMINISTIC_STATE_SAMPLER_

#include "ompl/base/StateSpace.h"
#include "ompl/base/StateSampler.h"
#include "ompl/base/samplers/deterministic/DeterministicSequence.h"

#include <memory>

namespace ompl
{
    namespace base
    {
        /**
       @anchor DeterministicStateSampler
       @par Short description
       \ref DeterministicStateSampler Implementation of a deterministic state sampler. The
       implementation allows to load and draw samples from a precomputed sequence or from the Halton Sequence,
       @par External documentation
       Dispertio: Optimal Sampling For Safe Deterministic Motion Planning
       L Palmieri, L Bruns, M Meurer, KO Arras - IEEE Robotics and Automation Letters, 2019
       DOI: [10.1109/LRA.2019.2958525](https://ieeexplore.ieee.org/abstract/document/8928532)<br>
       [[PDF]](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8928532)
    */

        /** \brief An abstract class for the concept of using deterministic sampling sequences
        to decrease the dispersion of the samples. */
        class DeterministicStateSampler : public StateSampler
        {
        public:
            enum DeterministicSamplerType
            {
                HALTON
            };

            /** \brief Constructor, which creates the sequence internally based on the specified sequence type.
            Uses the default constructor for the sequence.*/
            DeterministicStateSampler(const StateSpace *space,
                                      DeterministicSamplerType type = DeterministicSamplerType::HALTON);
            /** \brief Constructor that takes a pointer to a DeterministicSequence and uses that object instead
            of its own. This can be used to apply non default options to the deterministic sequence.*/
            DeterministicStateSampler(const StateSpace *space, std::shared_ptr<DeterministicSequence> sequence_ptr);

            virtual ~DeterministicStateSampler() = default;

            virtual void sampleUniform(State *state)
            {
                std::vector<double> sample = sequence_ptr_->sample();
                space_->copyFromReals(state, sample);
                return;
            }

            virtual void sampleUniformNear(State *, const State *, double)
            {
                OMPL_ERROR("sampleUniformNear is not supported for DeterministicStateSampler");
            }
            virtual void sampleGaussian(State *, const State *, double)
            {
                OMPL_ERROR("sampleGaussian is not supported for DeterministicStateSampler");
            }

        protected:
            std::shared_ptr<DeterministicSequence> sequence_ptr_;
        };

        /** \brief Deterministic state space sampler for SO(2) */
        class SO2DeterministicStateSampler : public DeterministicStateSampler
        {
        public:
            /** \brief Constructor, which creates the sequence internally based on the specified sequence type.
            Uses the default constructor for the sequence.*/
            SO2DeterministicStateSampler(const StateSpace *space,
                                         DeterministicSamplerType type = DeterministicSamplerType::HALTON)
              : DeterministicStateSampler(space, type)
            {
            }
            /** \brief Constructor that takes a pointer to a DeterministicSequence and uses that object instead
            of its own. This can be used to apply non default options to the deterministic sequence.*/
            SO2DeterministicStateSampler(const StateSpace *space, std::shared_ptr<DeterministicSequence> sequence_ptr)
              : DeterministicStateSampler(space, sequence_ptr)
            {
            }

            void sampleUniform(State *state) override;
            void sampleUniformNear(State *state, const State *near, double distance) override;
            void sampleGaussian(State *state, const State *mean, double stdDev) override;
        };

        /** \brief Deterministic state sampler for the R<sup>n</sup> state space */
        class RealVectorDeterministicStateSampler : public DeterministicStateSampler
        {
        public:
            /** \brief Constructor, which creates the sequence internally based on the specified sequence type.
            Uses the default constructor for the sequence.*/
            RealVectorDeterministicStateSampler(const StateSpace *space,
                                                DeterministicSamplerType type = DeterministicSamplerType::HALTON)
              : DeterministicStateSampler(space, type), stretch_(true)
            {
            }
            /** \brief Constructor that takes a pointer to a DeterministicSequence and uses that object instead
            of its own. This can be used to apply non default options to the deterministic sequence.*/
            RealVectorDeterministicStateSampler(const StateSpace *space,
                                                std::shared_ptr<DeterministicSequence> sequence_ptr,
                                                bool stretch = true)
              : DeterministicStateSampler(space, sequence_ptr), stretch_(stretch)
            {
            }

            void sampleUniform(State *state) override;
            void sampleUniformNear(State *state, const State *near, double distance) override;
            void sampleGaussian(State *state, const State *mean, double stdDev) override;

        private:
            bool stretch_{false};  // indicates whether the state is samples in [0,1] and should be stretched to the
                                   // state space boundaries
        };

        /** \brief Deterministic state sampler for the R<sup>n</sup> state space */
        class SE2DeterministicStateSampler : public DeterministicStateSampler
        {
        public:
            /** \brief Constructor, which creates the sequence internally based on the specified sequence type.
            Uses the default constructor for the sequence.*/
            SE2DeterministicStateSampler(const StateSpace *space,
                                         DeterministicSamplerType type = DeterministicSamplerType::HALTON)
              : DeterministicStateSampler(space, type), stretch_rv_(true), stretch_so2_(true)
            {
            }
            /** \brief Constructor that takes a pointer to a DeterministicSequence and uses that object instead
            of its own. This can be used to apply non default options to the deterministic sequence.*/
            SE2DeterministicStateSampler(const StateSpace *space, std::shared_ptr<DeterministicSequence> sequence_ptr,
                                         bool stretch_rv = true, bool stretch_so2 = true)
              : DeterministicStateSampler(space, sequence_ptr), stretch_rv_(stretch_rv), stretch_so2_(stretch_so2)
            {
            }

            void sampleUniform(State *state) override;
            void sampleUniformNear(State *state, const State *near, double distance) override;
            void sampleGaussian(State *state, const State *mean, double stdDev) override;

        private:
            bool stretch_rv_;   // indicates whether the xy state is sampled in [0,1] and should be stretched to the
                                // state space boundaries
            bool stretch_so2_;  // indicates whether the so2 state is sampled in [0,1] and should be stretched to
                                // [-pi;pi]
        };
    }  // namespace base
}  // namespace ompl

#endif
