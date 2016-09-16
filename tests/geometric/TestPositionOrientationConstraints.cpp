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

/* Author: Ryan Luna */

#define BOOST_TEST_MODULE "PositionOrientationConstraints"
#include <boost/math/constants/constants.hpp>
#include <boost/test/unit_test.hpp>

#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>

#include <ompl/geometric/constraints/OrientationConstraint.h>
#include <ompl/geometric/constraints/PositionConstraint.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

BOOST_AUTO_TEST_CASE(PositionConstraint)
{
    // Setup state space
    unsigned int dim = 3;
    ob::StateSpacePtr space(new ob::RealVectorStateSpace(dim));
    ob::RealVectorBounds bounds(dim);
    bounds.setLow(0);
    bounds.setHigh(1);

    space->as<ob::RealVectorStateSpace>()->setBounds(bounds);
    space->setup();

    // Setup constraint
    ob::StateSpace::SubstateLocation loc;
    loc.chain.clear();
    loc.space = space.get();

    ob::RealVectorBounds constraintBounds(dim);
    constraintBounds.setLow(0.25);
    constraintBounds.setHigh(0.75);

    og::PositionConstraint posConstraint(space, loc, constraintBounds);

    // Will sample this many states and project onto constraint if necessary
    unsigned int samples = 1000;
    ob::State *sampleState = space->allocState();
    ob::RealVectorStateSpace::StateType *rvState = sampleState->as<ob::RealVectorStateSpace::StateType>();

    ob::State *cloneState = space->allocState();
    ob::StateSamplerPtr stateSampler = space->allocStateSampler();
    for (unsigned int i = 0; i < samples; ++i)
    {
        stateSampler->sampleUniform(sampleState);
        bool satisfied = posConstraint.isSatisfied(sampleState);

        // Project the state.  It better be valid after that
        if (!satisfied)
        {
            // Make sure one of the values is actually unsatisfied first
            bool sat = true;
            for (unsigned int j = 0; j < dim; ++j)
                sat &= (rvState->values[j] >= 0.25 && rvState->values[j] <= 0.75);

            BOOST_CHECK(!sat);

            space->copyState(cloneState, sampleState);
            posConstraint.project(cloneState);
            BOOST_CHECK(posConstraint.isSatisfied(cloneState));

            // Copy cloneState into sample state to check satisfied state properties
            space->copyState(sampleState, cloneState);
        }

        // Make sure the values are actually satisfied
        for (unsigned int j = 0; j < dim; ++j)
        {
            BOOST_CHECK(rvState->values[j] >= 0.25);
            BOOST_CHECK(rvState->values[j] <= 0.75);
        }

        // If we project the state, we should get back the original state
        space->copyState(cloneState, sampleState);
        posConstraint.project(cloneState);
        BOOST_CHECK(space->equalStates(cloneState, sampleState));

        // The distance to constraint should be zero
        BOOST_CHECK(posConstraint.distance(sampleState) == 0.0);
    }

    // Sample states directly from constraint
    for (unsigned int i = 0; i < samples; ++i)
    {
        posConstraint.sample(sampleState);
        BOOST_CHECK(posConstraint.isSatisfied(sampleState));

        // Make sure the values are actually satisfied
        for (unsigned int j = 0; j < dim; ++j)
        {
            BOOST_CHECK(rvState->values[j] >= 0.25);
            BOOST_CHECK(rvState->values[j] <= 0.75);
        }
    }

    space->freeState(sampleState);
    space->freeState(cloneState);
}

BOOST_AUTO_TEST_CASE(OrientationConstraint)
{
    // Setup state space
    ob::StateSpacePtr space(new ob::SO3StateSpace());
    space->setup();

    // Setup constraint
    ob::StateSpace::SubstateLocation loc;
    loc.chain.clear();
    loc.space = space.get();

    ob::State *nominalOrientation = space->allocState();
    nominalOrientation->as<ob::SO3StateSpace::StateType>()->setIdentity();

    double tolerance = 30.0;  // tolerance of 30 degrees
    double TORAD = boost::math::constants::pi<double>() / 2.0;
    double tolRad = tolerance * TORAD;

    og::OrientationConstraint ornConstraint(space, loc, nominalOrientation, tolRad, tolRad, tolRad);

    // Will sample this many states and project onto constraint if necessary
    unsigned int samples = 1000;
    ob::State *sampleState = space->allocState();
    ob::State *cloneState = space->allocState();
    ob::StateSamplerPtr stateSampler = space->allocStateSampler();
    for (unsigned int i = 0; i < samples; ++i)
    {
        stateSampler->sampleUniform(sampleState);
        bool satisfied = ornConstraint.isSatisfied(sampleState);

        // Project the state.  It better be valid after that
        if (!satisfied)
        {
            space->copyState(cloneState, sampleState);
            ornConstraint.project(cloneState);
            BOOST_CHECK(ornConstraint.isSatisfied(cloneState));

            // Copy cloneState into sample state to check satisfied state properties
            space->copyState(sampleState, cloneState);
        }

        // If we project the state, we should get back the original state
        space->copyState(cloneState, sampleState);
        ornConstraint.project(cloneState);
        BOOST_CHECK(space->equalStates(cloneState, sampleState));
    }

    // Sample states directly from constraint
    for (unsigned int i = 0; i < samples; ++i)
    {
        ornConstraint.sample(sampleState);
        BOOST_CHECK(ornConstraint.isSatisfied(sampleState));
    }

    space->freeState(nominalOrientation);
    space->freeState(sampleState);
    space->freeState(cloneState);
}