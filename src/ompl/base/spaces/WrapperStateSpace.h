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

            virtual bool isCompound() const override
            {
                return space_->isCompound();
            };

            virtual bool isDiscrete() const override
            {
                return space_->isDiscrete();
            };

            virtual bool isHybrid() const override
            {
                return space_->isHybrid();
            };

            virtual bool isMetricSpace() const override
            {
                return space_->isMetricSpace();
            };

            virtual bool hasSymmetricDistance() const override
            {
                return space_->hasSymmetricDistance();
            };

            virtual bool hasSymmetricInterpolate() const override
            {
                return space_->hasSymmetricInterpolate();
            };

            virtual double getLongestValidSegmentFraction() const override
            {
                return space_->getLongestValidSegmentFraction();
            }

            virtual void setLongestValidSegmentFraction(double segmentFraction) override
            {
                space_->setLongestValidSegmentFraction(segmentFraction);
            }

            void setValidSegmentCountFactor(unsigned int factor)
            {
                space_->setValidSegmentCountFactor(factor);
            }

            unsigned int getValidSegmentCountFactor() const
            {
                return space_->getValidSegmentCountFactor();
            }

            virtual unsigned int validSegmentCount(const State *state1, const State *state2) const override
            {
                return space_->validSegmentCount(state1->as<StateType>()->getState(),
                                                 state2->as<StateType>()->getState());
            }

            virtual unsigned int getDimension() const override
            {
                return space_->getDimension();
            }

            virtual double getMaximumExtent() const override
            {
                return space_->getMaximumExtent();
            }

            virtual double getMeasure() const override
            {
                return space_->getMeasure();
            }

            virtual void enforceBounds(State *state) const override
            {
                space_->enforceBounds(state->as<StateType>()->getState());
            }

            virtual bool satisfiesBounds(const State *state) const override
            {
                return space_->satisfiesBounds(state->as<StateType>()->getState());
            }

            virtual void copyState(State *destination, const State *source) const override
            {
                space_->copyState(destination->as<StateType>()->getState(), source->as<StateType>()->getState());
            }

            State *cloneState(const State *source) const
            {
                StateType *clone = allocState()->as<StateType>();
                copyState(clone, source);
                return clone;
            }

            virtual double distance(const State *state1, const State *state2) const override
            {
                return space_->distance(state1->as<StateType>()->getState(), state2->as<StateType>()->getState());
            }

            virtual unsigned int getSerializationLength() const override
            {
                return space_->getSerializationLength();
            }

            virtual void serialize(void *serialization, const State *state) const override
            {
                space_->serialize(serialization, state->as<StateType>()->getState());
            }

            virtual void deserialize(State *state, const void *serialization) const override
            {
                space_->deserialize(state->as<StateType>()->getState(), serialization);
            }

            virtual bool equalStates(const State *state1, const State *state2) const override
            {
                return space_->equalStates(state1->as<StateType>()->getState(), state2->as<StateType>()->getState());
            }

            virtual void interpolate(const State *from, const State *to, double t, State *state) const override
            {
                return space_->interpolate(from->as<StateType>()->getState(), to->as<StateType>()->getState(), t,
                                           state->as<StateType>()->getState());
            }

            virtual StateSamplerPtr allocDefaultStateSampler() const override
            {
                return std::make_shared<WrapperStateSampler>(this, space_->allocDefaultStateSampler());
            }

            virtual State *allocState() const override
            {
                return new StateType(space_->allocState());
            }

            virtual void freeState(State *state) const override
            {
                StateType *wstate = state->as<StateType>();
                space_->freeState(wstate->getState());
                delete wstate;
            }

            virtual double *getValueAddressAtIndex(State *state, unsigned int index) const override
            {
                return space_->getValueAddressAtIndex(state->as<StateType>()->getState(), index);
            }

            virtual void copyToReals(std::vector<double> &reals, const State *source) const override
            {
                space_->copyToReals(reals, source->as<StateType>()->getState());
            }

            virtual void copyFromReals(State *destination, const std::vector<double> &reals) const override
            {
                space_->copyFromReals(destination->as<StateType>()->getState(), reals);
            }

            virtual void registerProjections() override
            {
                space_->registerProjections();
            }

            virtual void printState(const State *state, std::ostream &out = std::cout) const override
            {
                space_->printState(state->as<StateType>()->getState(), out);
            }

            virtual void printSettings(std::ostream &out) const override
            {
                space_->printSettings(out);
            }

            virtual void printProjections(std::ostream &out) const override
            {
                space_->printProjections(out);
            }

            virtual void sanityChecks(double zero, double eps, unsigned int flags) const override
            {
                space_->sanityChecks(zero, eps, flags);
            }

            virtual void sanityChecks() const override
            {
                space_->sanityChecks();
            }

            virtual StateSamplerPtr allocSubspaceStateSampler(const StateSpace *subspace) const override
            {
                return space_->allocSubspaceStateSampler(subspace);
            }

            virtual void computeLocations() override
            {
                space_->computeLocations();
            }

            virtual void setup() override
            {
                space_->setup();
            }

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
