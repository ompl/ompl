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

/* Author: Mark Moll, Ioan Sucan */

#include "ompl/base/spaces/SO3StateSpace.h"
#include <algorithm>
#include <limits>
#include <cmath>
#include "ompl/tools/config/MagicConstants.h"
#include <boost/math/constants/constants.hpp>
#include <boost/assert.hpp>

using namespace boost::math::double_constants;

static const double MAX_QUATERNION_NORM_ERROR = 1e-9;

/// @cond IGNORE
namespace ompl
{
    namespace base
    {
        static inline void computeAxisAngle(SO3StateSpace::StateType &q, double ax, double ay, double az, double angle)
        {
            double norm = std::sqrt(ax * ax + ay * ay + az * az);
            if (norm < MAX_QUATERNION_NORM_ERROR)
                q.setIdentity();
            else
            {
                double half_angle = angle / 2.0;
                double s = sin(half_angle) / norm;
                q.x = s * ax;
                q.y = s * ay;
                q.z = s * az;
                q.w = cos(half_angle);
            }
        }

        /* Standard quaternion multiplication: q = q0 * q1 */
        static inline void quaternionProduct(SO3StateSpace::StateType &q, const SO3StateSpace::StateType &q0,
                                             const SO3StateSpace::StateType &q1)
        {
            q.x = q0.w * q1.x + q0.x * q1.w + q0.y * q1.z - q0.z * q1.y;
            q.y = q0.w * q1.y + q0.y * q1.w + q0.z * q1.x - q0.x * q1.z;
            q.z = q0.w * q1.z + q0.z * q1.w + q0.x * q1.y - q0.y * q1.x;
            q.w = q0.w * q1.w - q0.x * q1.x - q0.y * q1.y - q0.z * q1.z;
        }

        inline double quaternionNormSquared(const SO3StateSpace::StateType &q)
        {
            return q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
        }
    }  // namespace base
}  // namespace ompl
/// @endcond

void ompl::base::SO3StateSpace::StateType::setAxisAngle(double ax, double ay, double az, double angle)
{
    computeAxisAngle(*this, ax, ay, az, angle);
}

void ompl::base::SO3StateSpace::StateType::setIdentity()
{
    x = y = z = 0.0;
    w = 1.0;
}

void ompl::base::SO3StateSampler::sampleUniform(State *state)
{
    rng_.quaternion(&state->as<SO3StateSpace::StateType>()->x);
}

void ompl::base::SO3StateSampler::sampleUniformNear(State *state, const State *near, const double distance)
{
    if (distance >= .25 * pi)
    {
        sampleUniform(state);
        return;
    }
    double d = rng_.uniform01();
    SO3StateSpace::StateType q, *qs = static_cast<SO3StateSpace::StateType *>(state);
    const auto *qnear = static_cast<const SO3StateSpace::StateType *>(near);
    computeAxisAngle(q, rng_.gaussian01(), rng_.gaussian01(), rng_.gaussian01(),
                     2. * pow(d, third) * distance);
    quaternionProduct(*qs, *qnear, q);
}

void ompl::base::SO3StateSampler::sampleGaussian(State *state, const State *mean, const double stdDev)
{
    // The standard deviation of the individual components of the tangent
    // perturbation needs to be scaled so that the expected quaternion distance
    // between the sampled state and the mean state is stdDev. The factor 2 is
    // due to the way we define distance (see also Matt Mason's lecture notes
    // on quaternions at
    // http://www.cs.cmu.edu/afs/cs/academic/class/16741-s07/www/lectures/Lecture8.pdf).
    // The 1/sqrt(3) factor is necessary because the distribution in the tangent
    // space is a 3-dimensional Gaussian, so that the *length* of a tangent
    // vector needs to be scaled by 1/sqrt(3).
    double rotDev = (2. * stdDev) / root_three;

    // CDF of N(0, 1.17) at -pi/4 is approx. .25, so there's .25 probability
    // weight in each tail. Since the maximum distance in SO(3) is pi/2, we're
    // essentially as likely to sample a state within distance [0, pi/4] as
    // within distance [pi/4, pi/2]. With most weight in the tails (that wrap
    // around in case of quaternions) we might as well sample uniformly.
    if (rotDev > 1.17)
    {
        sampleUniform(state);
        return;
    }

    double x = rng_.gaussian(0, rotDev), y = rng_.gaussian(0, rotDev), z = rng_.gaussian(0, rotDev),
           theta = std::sqrt(x * x + y * y + z * z);
    if (theta < std::numeric_limits<double>::epsilon())
        space_->copyState(state, mean);
    else
    {
        SO3StateSpace::StateType q, *qs = static_cast<SO3StateSpace::StateType *>(state);
        const auto *qmu = static_cast<const SO3StateSpace::StateType *>(mean);
        double half_theta = theta / 2.0;
        double s = sin(half_theta) / theta;
        q.w = cos(half_theta);
        q.x = s * x;
        q.y = s * y;
        q.z = s * z;
        quaternionProduct(*qs, *qmu, q);
    }
}

unsigned int ompl::base::SO3StateSpace::getDimension() const
{
    return 3;
}

double ompl::base::SO3StateSpace::getMaximumExtent() const
{
    return .5 * pi;
}

double ompl::base::SO3StateSpace::getMeasure() const
{
    // half of the surface area of a unit 3-sphere
    return pi * pi;
}

double ompl::base::SO3StateSpace::norm(const StateType *state) const
{
    double nrmSqr = quaternionNormSquared(*state);
    return (fabs(nrmSqr - 1.0) > std::numeric_limits<double>::epsilon()) ? std::sqrt(nrmSqr) : 1.0;
}

void ompl::base::SO3StateSpace::enforceBounds(State *state) const
{
    // see http://stackoverflow.com/questions/11667783/quaternion-and-normalization/12934750#12934750
    auto *qstate = static_cast<StateType *>(state);
    double nrmsq = quaternionNormSquared(*qstate);
    double error = std::abs(1.0 - nrmsq);
    const double epsilon = 2.107342e-08;
    if (error < epsilon)
    {
        double scale = 2.0 / (1.0 + nrmsq);
        qstate->x *= scale;
        qstate->y *= scale;
        qstate->z *= scale;
        qstate->w *= scale;
    }
    else
    {
        if (nrmsq < 1e-6)
            qstate->setIdentity();
        else
        {
            double scale = 1.0 / std::sqrt(nrmsq);
            qstate->x *= scale;
            qstate->y *= scale;
            qstate->z *= scale;
            qstate->w *= scale;
        }
    }
}

bool ompl::base::SO3StateSpace::satisfiesBounds(const State *state) const
{
    return fabs(norm(static_cast<const StateType *>(state)) - 1.0) < MAX_QUATERNION_NORM_ERROR;
}

void ompl::base::SO3StateSpace::copyState(State *destination, const State *source) const
{
    const auto *qsource = static_cast<const StateType *>(source);
    auto *qdestination = static_cast<StateType *>(destination);
    qdestination->x = qsource->x;
    qdestination->y = qsource->y;
    qdestination->z = qsource->z;
    qdestination->w = qsource->w;
}

unsigned int ompl::base::SO3StateSpace::getSerializationLength() const
{
    return sizeof(double) * 4;
}

void ompl::base::SO3StateSpace::serialize(void *serialization, const State *state) const
{
    memcpy(serialization, &state->as<StateType>()->x, sizeof(double) * 4);
}

void ompl::base::SO3StateSpace::deserialize(State *state, const void *serialization) const
{
    memcpy(&state->as<StateType>()->x, serialization, sizeof(double) * 4);
}

/// @cond IGNORE

/*
Based on code from :

Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/
*/
namespace ompl
{
    namespace base
    {
        static inline double arcLength(const State *state1, const State *state2)
        {
            const auto *qs1 = static_cast<const SO3StateSpace::StateType *>(state1);
            const auto *qs2 = static_cast<const SO3StateSpace::StateType *>(state2);
            double dq = fabs(qs1->x * qs2->x + qs1->y * qs2->y + qs1->z * qs2->z + qs1->w * qs2->w);
            if (dq > 1.0 - MAX_QUATERNION_NORM_ERROR)
                return 0.0;
            return acos(dq);
        }
    }  // namespace base
}  // namespace ompl
/// @endcond

double ompl::base::SO3StateSpace::distance(const State *state1, const State *state2) const
{
    BOOST_ASSERT_MSG(satisfiesBounds(state1) && satisfiesBounds(state2), "The states passed to SO3StateSpace::distance "
                                                                         "are not within bounds. Call "
                                                                         "SO3StateSpace::enforceBounds() in, e.g., "
                                                                         "ompl::control::ODESolver::"
                                                                         "PostPropagationEvent, "
                                                                         "ompl::control::StatePropagator, or "
                                                                         "ompl::base::StateValidityChecker");
    return arcLength(state1, state2);
}

bool ompl::base::SO3StateSpace::equalStates(const State *state1, const State *state2) const
{
    return arcLength(state1, state2) < std::numeric_limits<double>::epsilon();
}

/*
Based on code from :

Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/
*/
void ompl::base::SO3StateSpace::interpolate(const State *from, const State *to, const double t, State *state) const
{
    assert(fabs(norm(static_cast<const StateType *>(from)) - 1.0) < MAX_QUATERNION_NORM_ERROR);
    assert(fabs(norm(static_cast<const StateType *>(to)) - 1.0) < MAX_QUATERNION_NORM_ERROR);

    double theta = arcLength(from, to);
    if (theta > std::numeric_limits<double>::epsilon())
    {
        double d = 1.0 / sin(theta);
        double s0 = sin((1.0 - t) * theta);
        double s1 = sin(t * theta);

        const auto *qs1 = static_cast<const StateType *>(from);
        const auto *qs2 = static_cast<const StateType *>(to);
        auto *qr = static_cast<StateType *>(state);
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

ompl::base::StateSamplerPtr ompl::base::SO3StateSpace::allocDefaultStateSampler() const
{
    return std::make_shared<SO3StateSampler>(this);
}

ompl::base::State *ompl::base::SO3StateSpace::allocState() const
{
    return new StateType();
}

void ompl::base::SO3StateSpace::freeState(State *state) const
{
    delete static_cast<StateType *>(state);
}

void ompl::base::SO3StateSpace::registerProjections()
{
    class SO3DefaultProjection : public ProjectionEvaluator
    {
    public:
        SO3DefaultProjection(const StateSpace *space) : ProjectionEvaluator(space)
        {
        }

        unsigned int getDimension() const override
        {
            return 3;
        }

        void defaultCellSizes() override
        {
            cellSizes_.resize(3);
            cellSizes_[0] = pi / magic::PROJECTION_DIMENSION_SPLITS;
            cellSizes_[1] = pi / magic::PROJECTION_DIMENSION_SPLITS;
            cellSizes_[2] = pi / magic::PROJECTION_DIMENSION_SPLITS;
            bounds_.resize(3);
            bounds_.setLow(-1.0);
            bounds_.setHigh(1.0);
        }

        void project(const State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
        {
            projection(0) = state->as<SO3StateSpace::StateType>()->x;
            projection(1) = state->as<SO3StateSpace::StateType>()->y;
            projection(2) = state->as<SO3StateSpace::StateType>()->z;
        }
    };

    registerDefaultProjection(std::make_shared<SO3DefaultProjection>(this));
}

double *ompl::base::SO3StateSpace::getValueAddressAtIndex(State *state, const unsigned int index) const
{
    return index < 4 ? &(state->as<StateType>()->x) + index : nullptr;
}

void ompl::base::SO3StateSpace::printState(const State *state, std::ostream &out) const
{
    out << "SO3State [";
    if (state != nullptr)
    {
        const auto *qstate = static_cast<const StateType *>(state);
        out << qstate->x << " " << qstate->y << " " << qstate->z << " " << qstate->w;
    }
    else
        out << "nullptr";
    out << ']' << std::endl;
}

void ompl::base::SO3StateSpace::printSettings(std::ostream &out) const
{
    out << "SO(3) state space '" << getName() << "' (represented using quaternions)" << std::endl;
}
