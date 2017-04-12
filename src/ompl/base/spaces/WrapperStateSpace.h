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
                StateType() : state_(nullptr)
                {
                }

                State *getState() const
                {
                    return state_;
                }

                void setState(State *state)
                {
                    state_ = state;
                }

            protected:
                State *state_;
            };

            WrapperStateSpace(const StateSpacePtr space) : StateSpace(), space_(std::move(space))
            {
            }

            ~WrapperStateSpace() = default;

            const StateSpacePtr getSpace() const
            {
                return space_;
            }

            virtual bool isCompound() const
            {
                return space_->isCompound();
            };

            virtual bool isDiscrete() const
            {
                return space_->isDiscrete();
            };

            virtual bool isHybrid() const
            {
                return space_->isHybrid();
            };

            virtual bool isMetricSpace() const
            {
                return space_->isMetricSpace();
            };

            virtual bool hasSymmetricDistance() const
            {
                return space_->hasSymmetricDistance();
            };

            virtual bool hasSymmetricInterpolate() const
            {
                return space_->hasSymmetricInterpolate();
            };

            const std::string &getName() const
            {
                return space_->getName();
            };

            void setName(const std::string &name)
            {
                space_->setName(name);
            };

            int getType() const
            {
                return type_;
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

            const ParamSet &params() const
            {
                return space_->params();
            }

            virtual double getLongestValidSegmentFraction() const
            {
                return space_->getLongestValidSegmentFraction();
            }

            virtual void setLongestValidSegmentFraction(double segmentFraction)
            {
                space_->setLongestValidSegmentFraction(segmentFraction);
            }

            virtual unsigned int validSegmentCount(const State *state1, const State *state2) const
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

            virtual unsigned int getDimension() const
            {
                return space_->getDimension();
            }

            virtual double getMaximumExtent() const
            {
                return space_->getMaximumExtent();
            }

            virtual double getMeasure() const
            {
                return space_->getMeasure();
            }

            virtual void enforceBounds(State *state) const
            {
                space_->enforceBounds(state->as<StateType>()->getState());
            }

            virtual bool satisfiesBounds(const State *state) const
            {
                return space_->satisfiesBounds(state->as<StateType>()->getState());
            }

            virtual void copyState(State *destination, const State *source) const
            {
                space_->copyState(destination->as<StateType>()->getState(), source->as<StateType>()->getState());
            }

            State *cloneState(const State *source) const
            {
                StateType *clone = allocState()->as<StateType>();
                copyState(clone, source);
                return clone;
            }

            virtual double distance(const State *state1, const State *state2) const
            {
                return space_->distance(state1->as<StateType>()->getState(), state2->as<StateType>()->getState());
            }

            virtual unsigned int getSerializationLength() const
            {
                return space_->getSerializationLength();
            }

            virtual void serialize(void *serialization, const State *state) const
            {
                space_->serialize(serialization, state->as<StateType>()->getState());
            }

            virtual void deserialize(State *state, const void *serialization) const
            {
                space_->deserialize(state->as<StateType>()->getState(), serialization);
            }

            virtual bool equalStates(const State *state1, const State *state2) const
            {
                return space_->equalStates(state1->as<StateType>()->getState(), state2->as<StateType>()->getState());
            }

            virtual void interpolate(const State *from, const State *to, double t, State *state) const
            {
                return space_->interpolate(from->as<StateType>()->getState(), to->as<StateType>()->getState(), t,
                                           state->as<StateType>()->getState());
            }

            virtual StateSamplerPtr allocDefaultStateSampler() const
            {
                return StateSamplerPtr(new WrapperStateSampler(this, space_->allocDefaultStateSampler()));
            }

            virtual StateSamplerPtr allocStateSampler() const
            {
                return StateSamplerPtr(new WrapperStateSampler(this, space_->allocStateSampler()));
            }

            void setStateSamplerAllocator(const StateSamplerAllocator &ssa)
            {
                return space_->setStateSamplerAllocator(ssa);
            }

            void clearStateSamplerAllocator()
            {
                space_->clearStateSamplerAllocator();
            }

            virtual State *allocState() const
            {
                StateType *state = new StateType();
                state->setState(space_->allocState());
                return state;
            }

            virtual void freeState(State *state) const
            {
                StateType *wstate = state->as<StateType>();
                space_->freeState(wstate->getState());
                delete wstate;
            }

            virtual double *getValueAddressAtIndex(State *state, unsigned int index) const
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

            virtual void copyToReals(std::vector<double> &reals, const State *source) const override
            {
                space_->copyToReals(reals, source->as<StateType>()->getState());
            }

            virtual void copyFromReals(State *destination, const std::vector<double> &reals) const override
            {
                space_->copyFromReals(destination->as<StateType>()->getState(), reals);
            }

            void registerProjection(const std::string &name, const ProjectionEvaluatorPtr &projection)
            {
                space_->registerProjection(name, projection);
            }

            void registerDefaultProjection(const ProjectionEvaluatorPtr &projection)
            {
                space_->registerDefaultProjection(projection);
            }

            virtual void registerProjections()
            {
                space_->registerProjections();
            }

            ProjectionEvaluatorPtr getProjection(const std::string &name) const
            {
                return space_->getProjection(name);
            }

            ProjectionEvaluatorPtr getDefaultProjection() const
            {
                return space_->getDefaultProjection();
            }

            bool hasProjection(const std::string &name) const
            {
                return space_->hasProjection(name);
            }

            bool hasDefaultProjection() const
            {
                return space_->hasDefaultProjection();
            }

            const std::map<std::string, ProjectionEvaluatorPtr> &getRegisteredProjections() const
            {
                return space_->getRegisteredProjections();
            }

            virtual void printState(const State *state, std::ostream &out = std::cout) const
            {
                space_->printState(state->as<StateType>()->getState(), out);
            }

            virtual void printSettings(std::ostream &out) const
            {
                space_->printSettings(out);
            }

            virtual void printProjections(std::ostream &out) const
            {
                space_->printProjections(out);
            }

            virtual void sanityChecks(double zero, double eps, unsigned int flags) const
            {
                space_->sanityChecks(zero, eps, flags);
            }

            virtual void sanityChecks() const
            {
                space_->sanityChecks();
            }

            void diagram(std::ostream &out) const
            {
                space_->diagram(out);
            }

            void list(std::ostream &out) const
            {
                space_->list(out);
            }

            // static void Diagram(std::ostream &out)
            // {
            //     space_->Diagram(out);
            // }

            // static void List(std::ostream &out)
            // {
            //     space_->List(out);
            // }

            StateSamplerPtr allocSubspaceStateSampler(const StateSpacePtr &subspace) const
            {
                return space_->allocSubspaceStateSampler(subspace);
            }

            virtual StateSamplerPtr allocSubspaceStateSampler(const StateSpace *subspace) const
            {
                return space_->allocSubspaceStateSampler(subspace);
            }

            State *getSubstateAtLocation(State *state, const SubstateLocation &loc) const
            {
                return space_->getSubstateAtLocation(state->as<StateType>()->getState(), loc);
            }

            const State *getSubstateAtLocation(const State *state, const SubstateLocation &loc) const
            {
                return space_->getSubstateAtLocation(state, loc);
            }

            const std::map<std::string, SubstateLocation> &getSubstateLocationsByName() const
            {
                return space_->getSubstateLocationsByName();
            }

            void getCommonSubspaces(const StateSpacePtr &other, std::vector<std::string> &subspaces) const
            {
                return space_->getCommonSubspaces(other, subspaces);
            }

            void getCommonSubspaces(const StateSpace *other, std::vector<std::string> &subspaces) const
            {
                space_->getCommonSubspaces(other, subspaces);
            }

            virtual void computeLocations()
            {
                space_->computeLocations();
            }

            virtual void setup()
            {
                space_->setup();
            }

        protected:
            const StateSpacePtr space_;
        };
    }
}

#endif
