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

#include "ompl/base/StateSpace.h"

namespace ompl
{
    namespace base
    {
        class WrapperStateSampler : public StateSampler
        {
        public:
            WrapperStateSampler(const StateSpace *space, StateSamplerPtr sampler)
              : StateSampler(space), sampler_(std::move(sampler))
            {
            }

            void sampleUniform(State *state) override;
            void sampleUniformNear(State *state, const State *near, double distance) override;
            void sampleGaussian(State *state, const State *mean, double stdDev) override;

        protected:
            StateSamplerPtr sampler_;
        };

        class WrapperStateSpace : public StateSpace
        {
        public:
            class StateType : public State
            {
            public:
                StateType(State *state) : state_(state)
                {
                }

                const State *getState() const
                {
                    return state_;
                }

                State *getState()
                {
                    return state_;
                }

            protected:
                State *state_;
            };

            WrapperStateSpace(const StateSpacePtr space) : StateSpace(), space_(std::move(space))
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

            double getLongestValidSegmentFraction() const
            {
                return space_->getLongestValidSegmentFraction();
            }

            void setLongestValidSegmentFraction(double segmentFraction)
            {
                space_->setLongestValidSegmentFraction(segmentFraction);
            }

            unsigned int validSegmentCount(const State *state1, const State *state2) const
            {
                return space_->validSegmentCount(state1->as<StateType>()->getState(),
                                                 state2->as<StateType>()->getState());
            }

            void setValidSegmentCountFactor(unsigned int factor)
            {
                space_->setValidSegmentCountFactor(factor);
            }

            unsigned int getValidSegmentCountFactor() const
            {
                return space_->getValidSegmentCountFactor();
            }

            double getLongestValidSegmentLength() const
            {
                return space_->getLongestValidSegmentLength();
            }

            void computeSignature(std::vector<int> &signature) const
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

            State *cloneState(const State *source) const
            {
                StateType *clone = allocState()->as<StateType>();
                copyState(clone, source);
                return clone;
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
                StateType *wstate = state->as<StateType>();
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
