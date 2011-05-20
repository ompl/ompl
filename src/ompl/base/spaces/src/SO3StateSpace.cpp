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

#include "ompl/base/spaces/SO3StateSpace.h"
#include <algorithm>
#include <limits>
#include <cmath>
#include "ompl/util/MagicConstants.h"
#include <boost/math/constants/constants.hpp>

static const double MAX_QUATERNION_NORM_ERROR = 1e-9;

void ompl::base::SO3StateSpace::StateType::setAxisAngle(double ax, double ay, double az, double angle)
{
    double norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm < MAX_QUATERNION_NORM_ERROR)
        setIdentity();
    else
    {
        double s = sin(angle / 2.0);
        x = s * ax / norm;
        y = s * ay / norm;
        z = s * az / norm;
        w = cos(angle / 2.0);
    }
}

void ompl::base::SO3StateSpace::StateType::setIdentity(void)
{
    x = y = z = 0.0;
    w = 1.0;
}

void ompl::base::SO3StateSampler::sampleUniform(State *state)
{
    rng_.quaternion(&state->as<SO3StateSpace::StateType>()->x);
}

void ompl::base::SO3StateSampler::sampleUniformNear(State *state, const State * /* near */, const double /* distance */)
{
    /** \todo How do we sample near a quaternion ? */
    sampleUniform(state);
}

void ompl::base::SO3StateSampler::sampleGaussian(State *state, const State * /* mean */, const double /* stdDev */)
{
    /** \todo How do we sample quaternions using a Gaussian distribution ?*/
    sampleUniform(state);
}


unsigned int ompl::base::SO3StateSpace::getDimension(void) const
{
    return 3;
}

double ompl::base::SO3StateSpace::getMaximumExtent(void) const
{
    return boost::math::constants::pi<double>();
}

double ompl::base::SO3StateSpace::norm(const StateType *state) const
{
    double nrmSqr = state->x * state->x + state->y * state->y + state->z * state->z + state->w * state->w;
    return (fabs(nrmSqr - 1.0) > std::numeric_limits<double>::epsilon()) ? sqrt(nrmSqr) : 1.0;
}

void ompl::base::SO3StateSpace::enforceBounds(State *state) const
{
    StateType *qstate = static_cast<StateType*>(state);
    double nrm = norm(qstate);
    if (fabs(nrm - 1.0) > MAX_QUATERNION_NORM_ERROR)
    {
        qstate->x /= nrm;
        qstate->y /= nrm;
        qstate->z /= nrm;
        qstate->w /= nrm;
    }
    else
        qstate->setIdentity();
}

bool ompl::base::SO3StateSpace::satisfiesBounds(const State *state) const
{
    return fabs(norm(static_cast<const StateType*>(state)) - 1.0) < MAX_QUATERNION_NORM_ERROR;
}

void ompl::base::SO3StateSpace::copyState(State *destination, const State *source) const
{
    const StateType *qsource = static_cast<const StateType*>(source);
    StateType *qdestination = static_cast<StateType*>(destination);
    qdestination->x = qsource->x;
    qdestination->y = qsource->y;
    qdestination->z = qsource->z;
    qdestination->w = qsource->w;
}


/*
Based on code from :

Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/
*/
double ompl::base::SO3StateSpace::distance(const State *state1, const State *state2) const
{
    const StateType *qs1 = static_cast<const StateType*>(state1);
    const StateType *qs2 = static_cast<const StateType*>(state2);
    double dq = fabs(qs1->x * qs2->x + qs1->y * qs2->y + qs1->z * qs2->z + qs1->w * qs2->w);
    if (dq > 1.0 - MAX_QUATERNION_NORM_ERROR)
        return 0.0;
    else
        return acos(dq) * 2.0;
}

bool ompl::base::SO3StateSpace::equalStates(const State *state1, const State *state2) const
{
    return distance(state1, state2) < std::numeric_limits<double>::epsilon() * 2.0;
}

/*
Based on code from :

Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/
*/
void ompl::base::SO3StateSpace::interpolate(const State *from, const State *to, const double t, State *state) const
{
    assert(norm(static_cast<const StateType*>(from)) - 1.0 < MAX_QUATERNION_NORM_ERROR);
    assert(norm(static_cast<const StateType*>(to)) - 1.0 < MAX_QUATERNION_NORM_ERROR);

    double theta = distance(from, to) / 2.0;
    if (theta > std::numeric_limits<double>::epsilon())
    {
        double d = 1.0 / sin(theta);
        double s0 = sin((1.0 - t) * theta);
        double s1 = sin(t * theta);

        const StateType *qs1 = static_cast<const StateType*>(from);
        const StateType *qs2 = static_cast<const StateType*>(to);
        StateType       *qr  = static_cast<StateType*>(state);
        double dq = qs1->x * qs2->x + qs1->y * qs2->y + qs1->z * qs2->z + qs1->w * qs2->w;
        if (dq < 0)  // Take care of long angle case see http://en.wikipedia.org/wiki/Slerp
            s1 = -s1;

        qr->x = (qs1->x * s0 + qs2->x * s1) * d;
        qr->y = (qs1->y * s0 + qs2->y * s1) * d;
        qr->z = (qs1->z * s0 + qs2->z * s1) * d;
        qr->w = (qs1->w * s0 + qs2->w * s1) * d;
    }
    else
    {
        if (state != from)
            copyState(state, from);
    }
}

ompl::base::StateSamplerPtr ompl::base::SO3StateSpace::allocStateSampler(void) const
{
    return StateSamplerPtr(new SO3StateSampler(this));
}

ompl::base::State* ompl::base::SO3StateSpace::allocState(void) const
{
    return new StateType();
}

void ompl::base::SO3StateSpace::freeState(State *state) const
{
    delete static_cast<StateType*>(state);
}

void ompl::base::SO3StateSpace::registerProjections(void)
{
    class SO3DefaultProjection : public ProjectionEvaluator
    {
    public:

        SO3DefaultProjection(const StateSpace *space) : ProjectionEvaluator(space)
        {
        }

        virtual unsigned int getDimension(void) const
        {
            return 3;
        }

        virtual void defaultCellSizes(void)
        {
            cellSizes_.resize(3);
            cellSizes_[0] = boost::math::constants::pi<double>() / magic::PROJECTION_DIMENSION_SPLITS;
            cellSizes_[1] = boost::math::constants::pi<double>() / magic::PROJECTION_DIMENSION_SPLITS;
            cellSizes_[2] = boost::math::constants::pi<double>() / magic::PROJECTION_DIMENSION_SPLITS;
        }

        virtual void project(const State *state, EuclideanProjection &projection) const
        {
            projection.values[0] = state->as<SO3StateSpace::StateType>()->x;
            projection.values[1] = state->as<SO3StateSpace::StateType>()->y;
            projection.values[2] = state->as<SO3StateSpace::StateType>()->z;
        }
    };

    registerDefaultProjection(ProjectionEvaluatorPtr(dynamic_cast<ProjectionEvaluator*>(new SO3DefaultProjection(this))));
}

double* ompl::base::SO3StateSpace::getValueAddressAtIndex(State *state, const unsigned int index) const
{
    return index < 4 ? &(state->as<StateType>()->x) + index : NULL;
}

void ompl::base::SO3StateSpace::printState(const State *state, std::ostream &out) const
{
    out << "SO3State [";
    if (state)
    {
        const StateType *qstate = static_cast<const StateType*>(state);
        out << qstate->x << " " << qstate->y << " " << qstate->z << " " << qstate->w;
    }
    else
        out << "NULL";
    out << ']' << std::endl;
}

void ompl::base::SO3StateSpace::printSettings(std::ostream &out) const
{
    out << "SO(3) state space '" << getName() << "' (represented using quaternions)" << std::endl;
}
