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
                : cforest_(cforest), space_(space), planner_(NULL)
            {
                setName(space->getName() + "CForestWrapper");
            }

            ~CForestStateSpaceWrapper()
            {
            }

            void setPlanner(base::Planner *planner)
            {
                planner_ = planner;
            }

            const base::Planner* getPlanner() const
            {
                return planner_;
            }

            geometric::CForest* getCForestInstance() const
            {
                return cforest_;
            }

            virtual StateSamplerPtr allocDefaultStateSampler() const;

            virtual StateSamplerPtr allocStateSampler() const;

            virtual void setup();

            virtual bool isCompound() const
            {
                return space_->isCompound();
            }
            virtual bool isDiscrete() const
            {
                return space_->isDiscrete();
            }
            virtual bool isHybrid() const
            {
                return space_->isHybrid();
            }
            virtual bool isMetricSpace() const
            {
                return space_->isMetricSpace();
            }
            virtual bool hasSymmetricDistance() const
            {
                return space_->hasSymmetricDistance();
            }
            virtual bool hasSymmetricInterpolate() const
            {
                return space_->hasSymmetricInterpolate();
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
                return space_->validSegmentCount(state1, state2);
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
                space_->enforceBounds(state);
            }
            virtual bool satisfiesBounds(const State *state) const
            {
                return space_->satisfiesBounds(state);
            }
            virtual void copyState(State *destination, const State *source) const
            {
                space_->copyState(destination, source);
            }
            virtual double distance(const State *state1, const State *state2) const
            {
                return space_->distance(state1, state2);
            }
            virtual unsigned int getSerializationLength() const
            {
                return space_->getSerializationLength();
            }
            virtual void serialize(void *serialization, const State *state) const
            {
                space_->serialize(serialization, state);
            }
            virtual void deserialize(State *state, const void *serialization) const
            {
                space_->deserialize(state, serialization);
            }
            virtual bool equalStates(const State *state1, const State *state2) const
            {
                return space_->equalStates(state1, state2);
            }
            virtual void interpolate(const State *from, const State *to, const double t, State *state) const
            {
                space_->interpolate(from, to, t, state);
            }
            virtual State* allocState() const
            {
                return space_->allocState();
            }
            virtual void freeState(State *state) const
            {
                space_->freeState(state);
            }
            virtual double* getValueAddressAtIndex(State *state, const unsigned int index) const
            {
                return space_->getValueAddressAtIndex(state, index);
            }
            virtual void registerProjections()
            {
                space_->registerProjections();
            }
            virtual void printState(const State *state, std::ostream &out) const
            {
                space_->printState(state, out);
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
            virtual StateSamplerPtr allocSubspaceStateSampler(const StateSpace *subspace) const
            {
                return space_->allocSubspaceStateSampler(subspace);
            }
            virtual void computeLocations()
            {
                space_->computeLocations();
            }

        protected:
            geometric::CForest    *cforest_;
            StateSpace            *space_;
            Planner               *planner_;
        };
    }
}

#endif
