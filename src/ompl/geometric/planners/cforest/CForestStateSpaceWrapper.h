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

/* Authors: Mark Moll */

#ifndef OMPL_GEOMETRIC_PLANNERS_CFOREST_CFORESTSTATESPACEWRAPPER_
#define OMPL_GEOMETRIC_PLANNERS_CFOREST_CFORESTSTATESPACEWRAPPER_

#include "ompl/geometric/planners/cforest/CForestStateSampler.h"
#include "ompl/base/StateSpace.h"
#include "ompl/base/Planner.h"

namespace ompl
{
    namespace geometric
    {
        class CForest;
    }

    namespace base
    {
        /** \brief State space wrapper to use together with CForest. It adds some functionalities
           to the regular state spaces necessary to CForest. */
        class CForestStateSpaceWrapper : public StateSpace
        {
        public:
            CForestStateSpaceWrapper(geometric::CForest *cforest, base::StateSpace *space)
              : cforest_(cforest), space_(space), planner_(nullptr)
            {
                setName(space->getName() + "CForestWrapper");
            }

            ~CForestStateSpaceWrapper() override = default;

            void setPlanner(base::Planner *planner)
            {
                planner_ = planner;
            }

            const base::Planner *getPlanner() const
            {
                return planner_;
            }

            geometric::CForest *getCForestInstance() const
            {
                return cforest_;
            }

            StateSamplerPtr allocDefaultStateSampler() const override;

            StateSamplerPtr allocStateSampler() const override;

            void setup() override;

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
                return space_->validSegmentCount(state1, state2);
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
                space_->enforceBounds(state);
            }
            bool satisfiesBounds(const State *state) const override
            {
                return space_->satisfiesBounds(state);
            }
            void copyState(State *destination, const State *source) const override
            {
                space_->copyState(destination, source);
            }
            double distance(const State *state1, const State *state2) const override
            {
                return space_->distance(state1, state2);
            }
            unsigned int getSerializationLength() const override
            {
                return space_->getSerializationLength();
            }
            void serialize(void *serialization, const State *state) const override
            {
                space_->serialize(serialization, state);
            }
            void deserialize(State *state, const void *serialization) const override
            {
                space_->deserialize(state, serialization);
            }
            bool equalStates(const State *state1, const State *state2) const override
            {
                return space_->equalStates(state1, state2);
            }
            void interpolate(const State *from, const State *to, const double t, State *state) const override
            {
                space_->interpolate(from, to, t, state);
            }
            State *allocState() const override
            {
                return space_->allocState();
            }
            void freeState(State *state) const override
            {
                space_->freeState(state);
            }
            double *getValueAddressAtIndex(State *state, const unsigned int index) const override
            {
                return space_->getValueAddressAtIndex(state, index);
            }
            void registerProjections() override
            {
                space_->registerProjections();
            }
            void printState(const State *state, std::ostream &out) const override
            {
                space_->printState(state, out);
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

        protected:
            geometric::CForest *cforest_;
            StateSpace *space_;
            Planner *planner_;
        };
    }
}

#endif
