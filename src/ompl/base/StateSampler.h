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

/* Author: Ioan Sucan */

#ifndef OMPL_BASE_STATE_SAMPLER_
#define OMPL_BASE_STATE_SAMPLER_

#include "ompl/base/State.h"
#include "ompl/util/RandomNumbers.h"
#include "ompl/util/ClassForward.h"
#include <vector>
#include <string>
#include <functional>

namespace ompl
{
    namespace base
    {
        /// @cond IGNORE
        OMPL_CLASS_FORWARD(StateSpace);
        /// @endcond

        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::StateSampler */
        OMPL_CLASS_FORWARD(StateSampler);
        /// @endcond

        /** \class ompl::base::StateSamplerPtr
            \brief A shared pointer wrapper for ompl::base::StateSampler */

        /** \brief Abstract definition of a state space sampler. */
        class StateSampler
        {
        public:
            // non-copyable
            StateSampler(const StateSampler &) = delete;
            StateSampler &operator=(const StateSampler &) = delete;

            /** \brief Constructor */
            StateSampler(const StateSpace *space) : space_(space)
            {
            }

            virtual ~StateSampler() = default;

            /** \brief Sample a state */
            virtual void sampleUniform(State *state) = 0;

            /** \brief Sample a state near another, within a neighborhood controlled by a distance parameter.

            Typically, StateSampler-derived classes will return in `state` a
            state that is uniformly distributed within a ball with radius
            `distance` defined by the distance function from the corresponding
            state space. However, this is not guaranteed. For example, the
            default state sampler for the RealVectorStateSpace returns samples
            uniformly distributed using L_inf distance, while the default
            distance function is L_2 distance.
            */
            virtual void sampleUniformNear(State *state, const State *near, double distance) = 0;

            /** \brief Sample a state using a Gaussian distribution with given \e mean and standard deviation (\e
            stdDev).

            As with sampleUniform, the implementation of sampleGaussian is
            specific to the derived class and few assumptions can be made
            about the distance between `state` and `mean`.
            */
            virtual void sampleGaussian(State *state, const State *mean, double stdDev) = 0;

            /** \brief Set the seed of the underlying RNG */
            void setLocalSeed(std::uint_fast32_t localSeed)
            {
                rng_.setLocalSeed(localSeed);
            };

        protected:
            /** \brief The state space this sampler samples */
            const StateSpace *space_;

            /** \brief An instance of a random number generator */
            RNG rng_;
        };

        /** \brief Definition of a compound state sampler. This is useful to construct samplers for compound states. */
        class CompoundStateSampler : public StateSampler
        {
        public:
            /** \brief Constructor */
            CompoundStateSampler(const StateSpace *space) : StateSampler(space), samplerCount_(0)
            {
            }

            /** \brief Destructor. This frees the added samplers as well. */
            ~CompoundStateSampler() override = default;

            /** \brief Add a sampler as part of the new compound
                sampler. This sampler is used to sample part of the
                compound state. When sampling near a state, the
                compound sampler calls in to added samplers. The
                distance passed to the called samplers is adjusted
                according to the specified importance. */
            virtual void addSampler(const StateSamplerPtr &sampler, double weightImportance);

            void sampleUniform(State *state) override;

            /** \brief Call sampleUniformNear for each of the subspace states
                with distance scaled by the corresponding subspace weight. */
            void sampleUniformNear(State *state, const State *near, double distance) override;

            /** \brief Call sampleGaussian for each of the subspace states
                with stdDev scaled by the corresponding subspace weight. */
            void sampleGaussian(State *state, const State *mean, double stdDev) override;

        protected:
            /** \brief The samplers that are composed */
            std::vector<StateSamplerPtr> samplers_;

            /** \brief The weight of each sampler (used when sampling near a state) */
            std::vector<double> weightImportance_;

        private:
            /** \brief The number of samplers that are composed */
            unsigned int samplerCount_;
        };

        /** \brief Construct a sampler that samples only within a subspace of the space */
        class SubspaceStateSampler : public StateSampler
        {
        public:
            /** \brief Construct a sampler for \e space but only sample components common to \e subspace. Use \e weight
             * as a multiplicative factor for \e distance and \e stdDev in the sampleUniformNear() and sampleGaussian()
             * functions. */
            SubspaceStateSampler(const StateSpace *space, const StateSpace *subspace, double weight);
            ~SubspaceStateSampler() override;

            void sampleUniform(State *state) override;

            void sampleUniformNear(State *state, const State *near, double distance) override;

            void sampleGaussian(State *state, const State *mean, double stdDev) override;

        protected:
            /** \brief The subspace to sample */
            const StateSpace *subspace_;

            /** \brief The sampler for the subspace */
            StateSamplerPtr subspaceSampler_;

            /** \brief The weigth factor to multiply distance and stdDev when sampling in the vicinity of a state */
            double weight_;

            /** \brief The names of common subspaces between \e space_ and \e subspace_; these are the ones copied after
             * sampling a state */
            std::vector<std::string> subspaces_;

        private:
            /** \brief Temporary work area */
            State *work_;

            /** \brief Temporary work area */
            State *work2_;
        };

        /** \brief Definition of a function that can allocate a state sampler */
        using StateSamplerAllocator = std::function<StateSamplerPtr(const StateSpace *)>;
    }
}

#endif
