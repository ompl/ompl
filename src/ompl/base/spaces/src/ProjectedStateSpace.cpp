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

#include "ompl/base/spaces/ProjectedStateSpace.h"

#include "ompl/base/PlannerDataGraph.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/util/Exception.h"

#include <boost/graph/iteration_macros.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

/// ProjectedStateSampler

/// Public

ompl::base::ProjectedStateSampler::ProjectedStateSampler(const SpaceInformation *si)
  : RealVectorStateSampler(si->getStateSpace().get()), ss_(*si->getStateSpace()->as<ProjectedStateSpace>())
{
    ProjectedStateSpace::checkSpace(si);
}

ompl::base::ProjectedStateSampler::ProjectedStateSampler(const ProjectedStateSpace &ss)
  : RealVectorStateSampler(&ss), ss_(ss)
{
}

void ompl::base::ProjectedStateSampler::sampleUniform(State *state)
{
    RealVectorStateSampler::sampleUniform(state);
    ss_.getConstraint()->project(state);
}

void ompl::base::ProjectedStateSampler::sampleUniformNear(State *state, const State *near, const double distance)
{
    RealVectorStateSampler::sampleUniformNear(state, near, distance);
    ss_.getConstraint()->project(state);
}

void ompl::base::ProjectedStateSampler::sampleGaussian(State *state, const State *mean, const double stdDev)
{
    RealVectorStateSampler::sampleGaussian(state, mean, stdDev);
    ss_.getConstraint()->project(state);
}

/// ProjectedValidStateSampler

/// Public

ompl::base::ProjectedValidStateSampler::ProjectedValidStateSampler(const SpaceInformation *si)
  : ValidStateSampler(si)
  , sampler_(si)
  , constraint_(si->getStateSpace()->as<ompl::base::ProjectedStateSpace>()->getConstraint())
  , scratch_(si->allocState())
{
    ProjectedStateSpace::checkSpace(si);
}

ompl::base::ProjectedValidStateSampler::~ProjectedValidStateSampler()
{
    si_->freeState(scratch_);
}

bool ompl::base::ProjectedValidStateSampler::sample(State *state)
{
    // Rejection sample for at most attempts_ tries.
    unsigned int tries = 0;
    bool valid;
    si_->copyState(scratch_, state);
    double dist = si_->getSpaceMeasure();

    do
    {
        sampler_.sampleUniformNear(state, scratch_, dist);
        dist *= 0.5;
    } while (!(valid = si_->isValid(state) && constraint_->isSatisfied(state)) && ++tries < attempts_);

    return valid;
}

bool ompl::base::ProjectedValidStateSampler::sampleNear(State *state, const State *near, const double distance)
{
    // Rejection sample for at most attempts_ tries.
    unsigned int tries = 0;
    bool valid;
    do
        sampler_.sampleUniformNear(state, near, distance);
    while (!(valid = si_->isValid(state) && constraint_->isSatisfied(state)) && ++tries < attempts_);

    return valid;
}


/// ProjectedStateSpace

/// Public

void ompl::base::ProjectedStateSpace::checkSpace(const SpaceInformation *si)
{
    if (!dynamic_cast<ProjectedStateSpace *>(si->getStateSpace().get()))
        throw ompl::Exception("ompl::base::ProjectedStateSpace(): "
                              "si needs to use an ProjectedStateSpace!");
}

bool ompl::base::ProjectedStateSpace::traverseManifold(const State *from, const State *to, const bool interpolate,
                                                       std::vector<State *> *stateList) const
{
    // number of discrete steps between a and b in the state space
    int n = validSegmentCount(from, to);

    // Save a copy of the from state.
    if (stateList)
    {
        stateList->clear();
        stateList->push_back(si_->cloneState(from)->as<State>());
    }

    if (n == 0)  // don't divide by zero
        return true;

    if (!constraint_->isSatisfied(from))
        return false;

    const StateValidityCheckerPtr &svc = si_->getStateValidityChecker();
    double dist = distance(from, to);

    State *previous = cloneState(from);
    State *scratch = allocState();

    bool there = false;
    while (!(there = dist < (delta_ + std::numeric_limits<double>::epsilon())))
    {
        // Compute the parameterization for interpolation
        double t = delta_ / dist;
        RealVectorStateSpace::interpolate(previous, to, t, scratch);

        // Project new state onto constraint manifold
        bool onManifold = constraint_->project(scratch);

        // Make sure the new state is valid, or we don't care as we are simply interpolating
        bool valid = interpolate || svc->isValid(scratch);

        // Check if we have deviated too far from our previous state
        bool deviated = distance(previous, scratch) > 2.0 * delta_;

        if (!onManifold || !valid || deviated)
            break;

        // Store the new state
        if (stateList)
            stateList->push_back(si_->cloneState(scratch)->as<State>());

        // Check for divergence. Divergence is declared if we are no closer than
        // before projection
        double newDist = distance(scratch, to);
        if (newDist >= dist)
            break;

        dist = newDist;
        copyState(previous, scratch);
    }

    if (there && stateList)
        stateList->push_back(si_->cloneState(to)->as<State>());

    freeState(scratch);
    freeState(previous);
    return there;
}

ompl::base::StateSamplerPtr ompl::base::ProjectedStateSpace::allocDefaultStateSampler() const
{
    return StateSamplerPtr(new ProjectedStateSampler(*this));
}
