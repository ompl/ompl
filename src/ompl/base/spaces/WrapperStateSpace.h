/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2017, Rice University
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

/* Author: Zachary Kingston */

#ifndef OMPL_BASE_SPACES_WRAPPER_STATE_SPACE_
#define OMPL_BASE_SPACES_WRAPPER_STATE_SPACE_

#include <utility>

#include "ompl/base/StateSpace.h"

namespace ompl
{
    namespace base
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::WrapperStateSpace */
        OMPL_CLASS_FORWARD(WrapperStateSpace);
        /// @endcond

        /** \brief A state sampler that wraps around another state sampler. */
        class WrapperStateSampler : public StateSampler
        {
        public:
            /** \brief Constructor. Requires the wrapper state space \a space
             * and the underlying sampler \a sampler. */
            WrapperStateSampler(const StateSpace *space, StateSamplerPtr sampler)
              : StateSampler(space), sampler_(std::move(sampler))
            {
            }

            /** \brief Sample a state using underlying sampler. */
            void sampleUniform(State *state) override;

            /** \brief Sample a nearby state using underlying sampler. */
            void sampleUniformNear(State *state, const State *near, double distance) override;

            /** \brief Sample a state within a Gaussian distribution using underlying sampler. */
            void sampleGaussian(State *state, const State *mean, double stdDev) override;

        protected:
            /** \brief Underlying state sampler. */
            StateSamplerPtr sampler_;
        };

        /** \brief A projection evaluator that wraps around another projection
         * evaluator. */
        class WrapperProjectionEvaluator : public ProjectionEvaluator
        {
        public:
            WrapperProjectionEvaluator(const WrapperStateSpace *space);

            void setup() override;

            unsigned int getDimension() const override;

            void project(const State *state, Eigen::Ref<Eigen::VectorXd> projection) const override;

        private:
            /** \brief Projection from wrapped space. */
            ProjectionEvaluatorPtr projection_;
        };

        /** \brief State space wrapper that transparently passes state space
         * operations through to the underlying space. Allows augmentation of
         * state spaces with additional information. */
        class WrapperStateSpace : public StateSpace
        {
        public:
            /** \brief Wrapper state type. Contains a reference to an underlying state. */
            class StateType : public State
            {
            public:
                /** \brief Constructor. Takes a reference \a state to the underlying state. */
                StateType(State *state) : state_(state)
                {
                }

                /** \brief Get a const pointer to the underlying state. */
                const State *getState() const
                {
                    return state_;
                }

                /** \brief Get a pointer to the underlying state. */
                State *getState()
                {
                    return state_;
                }

            protected:
                /** \brief Underlying state. */
                State *state_;
            };

            WrapperStateSpace(const StateSpacePtr &space) : space_(space)
            {
            }

            bool isCompound() const override
            {
                return space_->isCompound();
            }

            bool isDiscrete() const override
            {
                return space_->isDiscrete();
            }

            bool isHybrid() const override
            {
                return space_->isHybrid();
            }

            bool isMetricSpace() const override
            {
                return space_->isMetricSpace();
            }

            bool hasSymmetricDistance() const override
            {
                return space_->hasSymmetricDistance();
            }

            bool hasSymmetricInterpolate() const override
            {
                return space_->hasSymmetricInterpolate();
            }

            const std::string &getName() const
            {
                return space_->getName();
            }

            void setName(const std::string &name)
            {
                space_->setName(name);
            }

            int getType() const
            {
                return space_->getType();
            }

            bool includes(const StateSpacePtr &other) const
            {
                return space_->includes(other);
            }

            bool includes(const StateSpace *other) const
            {
                return space_->includes(other);
            }

            bool covers(const StateSpacePtr &other) const
            {
                return space_->covers(other);
            }

            bool covers(const StateSpace *other) const
            {
                return space_->covers(other);
            }

            ParamSet &params()
            {
                return space_->params();
            }

            /** \brief Get the parameters for this space */
            const ParamSet &params() const
            {
                return space_->params();
            }

            double getLongestValidSegmentFraction() const override
            {
                return space_->getLongestValidSegmentFraction();
            }

            void setLongestValidSegmentFraction(double segmentFraction) override
            {
                space_->setLongestValidSegmentFraction(segmentFraction);
            }

            unsigned int validSegmentCount(const State *state1, const State *state2) const override
            {
                return space_->validSegmentCount(state1->as<StateType>()->getState(),
                                                 state2->as<StateType>()->getState());
            }

            void setValidSegmentCountFactor(unsigned int factor) override
            {
                space_->setValidSegmentCountFactor(factor);
            }

            unsigned int getValidSegmentCountFactor() const override
            {
                return space_->getValidSegmentCountFactor();
            }

            double getLongestValidSegmentLength() const override
            {
                return space_->getLongestValidSegmentLength();
            }

            void computeSignature(std::vector<int> &signature) const override
            {
                space_->computeSignature(signature);
            }

            unsigned int getDimension() const override
            {
                return space_->getDimension();
            }

            double getMaximumExtent() const override
            {
                return space_->getMaximumExtent();
            }

            double getMeasure() const override
            {
                return space_->getMeasure();
            }

            void enforceBounds(State *state) const override
            {
                space_->enforceBounds(state->as<StateType>()->getState());
            }

            bool satisfiesBounds(const State *state) const override
            {
                return space_->satisfiesBounds(state->as<StateType>()->getState());
            }

            void copyState(State *destination, const State *source) const override
            {
                space_->copyState(destination->as<StateType>()->getState(), source->as<StateType>()->getState());
            }

            double distance(const State *state1, const State *state2) const override
            {
                return space_->distance(state1->as<StateType>()->getState(), state2->as<StateType>()->getState());
            }

            unsigned int getSerializationLength() const override
            {
                return space_->getSerializationLength();
            }

            void serialize(void *serialization, const State *state) const override
            {
                space_->serialize(serialization, state->as<StateType>()->getState());
            }

            void deserialize(State *state, const void *serialization) const override
            {
                space_->deserialize(state->as<StateType>()->getState(), serialization);
            }

            bool equalStates(const State *state1, const State *state2) const override
            {
                return space_->equalStates(state1->as<StateType>()->getState(), state2->as<StateType>()->getState());
            }

            void interpolate(const State *from, const State *to, double t, State *state) const override
            {
                return space_->interpolate(from->as<StateType>()->getState(), to->as<StateType>()->getState(), t,
                                           state->as<StateType>()->getState());
            }

            StateSamplerPtr allocDefaultStateSampler() const override
            {
                return std::make_shared<WrapperStateSampler>(this, space_->allocDefaultStateSampler());
            }

            State *allocState() const override
            {
                return new StateType(space_->allocState());
            }

            void freeState(State *state) const override
            {
                auto *wstate = state->as<StateType>();
                space_->freeState(wstate->getState());
                delete wstate;
            }

            double *getValueAddressAtIndex(State *state, unsigned int index) const override
            {
                return space_->getValueAddressAtIndex(state->as<StateType>()->getState(), index);
            }

            const double *getValueAddressAtIndex(const State *state, unsigned int index) const
            {
                return space_->getValueAddressAtIndex(state->as<StateType>()->getState(), index);
            }

            const std::vector<ValueLocation> &getValueLocations() const
            {
                return space_->getValueLocations();
            }

            const std::map<std::string, ValueLocation> &getValueLocationsByName() const
            {
                return space_->getValueLocationsByName();
            }

            double *getValueAddressAtLocation(State *state, const ValueLocation &loc) const
            {
                return space_->getValueAddressAtLocation(state->as<StateType>()->getState(), loc);
            }

            const double *getValueAddressAtLocation(const State *state, const ValueLocation &loc) const
            {
                return space_->getValueAddressAtLocation(state->as<StateType>()->getState(), loc);
            }

            double *getValueAddressAtName(State *state, const std::string &name) const
            {
                return space_->getValueAddressAtName(state->as<StateType>()->getState(), name);
            }

            const double *getValueAddressAtName(const State *state, const std::string &name) const
            {
                return space_->getValueAddressAtName(state->as<StateType>()->getState(), name);
            }

            void copyToReals(std::vector<double> &reals, const State *source) const override
            {
                space_->copyToReals(reals, source->as<StateType>()->getState());
            }

            void copyFromReals(State *destination, const std::vector<double> &reals) const override
            {
                space_->copyFromReals(destination->as<StateType>()->getState(), reals);
            }

            void registerProjections() override
            {
                space_->registerProjections();
            }

            void printState(const State *state, std::ostream &out = std::cout) const override
            {
                space_->printState(state->as<StateType>()->getState(), out);
            }

            void printSettings(std::ostream &out) const override
            {
                space_->printSettings(out);
            }

            void printProjections(std::ostream &out) const override
            {
                space_->printProjections(out);
            }

            void sanityChecks(double zero, double eps, unsigned int flags) const override
            {
                space_->sanityChecks(zero, eps, flags);
            }

            void sanityChecks() const override
            {
                space_->sanityChecks();
            }

            StateSamplerPtr allocSubspaceStateSampler(const StateSpace *subspace) const override
            {
                return space_->allocSubspaceStateSampler(subspace);
            }

            void computeLocations() override
            {
                space_->computeLocations();
            }

            void setup() override;

            const StateSpacePtr &getSpace() const
            {
                return space_;
            }

        protected:
            const StateSpacePtr space_;
        };
    }
}

#endif
