/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Rice University
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
 *   * Neither the name of Rice University nor the names of its
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

#include "PlanarManipulatorStateSpace.h"
#include "PlanarManipulatorProjectionEvaluator.h"
#include <boost/math/constants/constants.hpp>

#ifndef PI
#define PI boost::math::constants::pi<double>()
#define TWOPI boost::math::constants::two_pi<double>()
#endif

PlanarManipulatorStateSampler::PlanarManipulatorStateSampler(const ompl::base::StateSpace *space)
  : ompl::base::StateSampler(space)
{
}

void PlanarManipulatorStateSampler::sampleUniform(ompl::base::State *state)
{
    double *values = state->as<PlanarManipulatorStateSpace::StateType>()->values;
    for (unsigned int i = 0; i < space_->getDimension(); ++i)
        values[i] = rng_.uniformReal(-PI, PI);
}

void PlanarManipulatorStateSampler::sampleUniformNear(ompl::base::State *state, const ompl::base::State *near,
                                                      const double distance)
{
    double *values = state->as<PlanarManipulatorStateSpace::StateType>()->values;
    const double *nearVals = near->as<PlanarManipulatorStateSpace::StateType>()->values;
    for (unsigned int i = 0; i < space_->getDimension(); ++i)
        values[i] = rng_.uniformReal(nearVals[i] - distance, nearVals[i] + distance);

    space_->enforceBounds(state);
}

void PlanarManipulatorStateSampler::sampleGaussian(ompl::base::State *state, const ompl::base::State *mean,
                                                   const double stdDev)
{
    const double *meanVals = mean->as<PlanarManipulatorStateSpace::StateType>()->values;
    double *values = state->as<PlanarManipulatorStateSpace::StateType>()->values;
    for (unsigned int i = 0; i < space_->getDimension(); ++i)
        values[i] = rng_.gaussian(meanVals[i], stdDev);

    space_->enforceBounds(state);
}

PlanarManipulatorStateSpace::PlanarManipulatorStateSpace(const PlanarManipulator *manip)
  : ompl::base::StateSpace(), manip_(manip), dimension_(manip_->getNumLinks()), cartInterpolator_(false)
{
    type_ = ompl::base::STATE_SPACE_PLANAR_MANIPULATOR;
    setName("PlanarManipulator");
}

PlanarManipulatorStateSpace::~PlanarManipulatorStateSpace()
{
}

bool PlanarManipulatorStateSpace::usingCartesianInterpolator() const
{
    return cartInterpolator_;
}

void PlanarManipulatorStateSpace::useCartesianInterpolator(bool useIt)
{
    cartInterpolator_ = useIt;
}

bool PlanarManipulatorStateSpace::isMetricSpace() const
{
    return true;
}

bool PlanarManipulatorStateSpace::hasSymmetricDistance() const
{
    return true;
}

bool PlanarManipulatorStateSpace::hasSymmetricInterpolate() const
{
    return true;
}

unsigned int PlanarManipulatorStateSpace::getDimension() const
{
    return dimension_;
}

double PlanarManipulatorStateSpace::getMaximumExtent() const
{
    return dimension_ * PI;
}

double PlanarManipulatorStateSpace::getMeasure() const
{
    return dimension_ * TWOPI;
}

void PlanarManipulatorStateSpace::enforceBounds(ompl::base::State *state) const
{
    for (unsigned int i = 0; i < dimension_; ++i)
    {
        double v = fmod(state->as<StateType>()->values[i], TWOPI);
        if (v < -boost::math::constants::pi<double>())
            v += 2.0 * boost::math::constants::pi<double>();
        else if (v >= boost::math::constants::pi<double>())
            v -= 2.0 * boost::math::constants::pi<double>();
        state->as<StateType>()->values[i] = v;
    }
}

bool PlanarManipulatorStateSpace::satisfiesBounds(const ompl::base::State *state) const
{
    bool good = true;
    for (unsigned int i = 0; i < dimension_ && good; ++i)
        good = state->as<StateType>()->values[i] < PI && state->as<StateType>()->values[i] >= -PI;

    return good;
}

void PlanarManipulatorStateSpace::copyState(ompl::base::State *dest, const ompl::base::State *src) const
{
    memcpy(static_cast<StateType *>(dest)->values, static_cast<const StateType *>(src)->values,
           dimension_ * sizeof(double));
}

double PlanarManipulatorStateSpace::distance(const ompl::base::State *s1, const ompl::base::State *s2) const
{
    if (cartInterpolator_)
    {
        // cartesian distance
        const double *v1 = s1->as<StateType>()->values;
        const double *v2 = s2->as<StateType>()->values;

        std::vector<Eigen::Affine2d> f1, f2;
        manip_->FK(v1, f1);
        manip_->FK(v2, f2);

        double dist = 0.0;
        for (size_t i = 0; i < f1.size(); ++i)
        {
            dist += (f1[i].translation() - f2[i].translation()).norm();
        }
        return dist;
    }
    else
    {
        BOOST_ASSERT_MSG(satisfiesBounds(s1) && satisfiesBounds(s2), "The states passed to "
                                                                     "PlanarManipulatorStateSpace::distance are not "
                                                                     "within bounds. Call "
                                                                     "PlanarManipulatorStateSpace::enforceBounds() in, "
                                                                     "e.g., "
                                                                     "ompl::control::ODESolver::PostPropagationEvent, "
                                                                     "ompl::control::StatePropagator, or "
                                                                     "ompl::base::StateValidityChecker");

        const double *v1 = s1->as<StateType>()->values;
        const double *v2 = s2->as<StateType>()->values;
        double d = 0.0;

        for (unsigned int i = 0; i < dimension_; ++i)
        {
            double dt = std::abs(v1[i] - v2[i]);
            d += (dt > PI ? TWOPI - dt : dt);
        }

        return d;
    }
}

bool PlanarManipulatorStateSpace::equalStates(const ompl::base::State *s1, const ompl::base::State *s2) const
{
    const double *v1 = s1->as<StateType>()->values;
    const double *v2 = s2->as<StateType>()->values;

    double twoeps = std::numeric_limits<double>::epsilon() * 2.0;
    for (unsigned int i = 0; i < dimension_; ++i)
    {
        if (fabs(v1[i] - v2[i]) > twoeps)
            return false;
    }

    return true;
}

void PlanarManipulatorStateSpace::interpolate(const ompl::base::State *from, const ompl::base::State *to,
                                              const double t, ompl::base::State *state) const
{
    const double *f = from->as<StateType>()->values;
    const double *too = to->as<StateType>()->values;
    double *out = state->as<StateType>()->values;

    if (cartInterpolator_)
    {
        manip_->cartesianInterpolate(f, too, t, out);
    }
    else
    {
        for (unsigned int i = 0; i < dimension_; ++i)
            linearInterpolate(f[i], too[i], t, out[i]);
    }
}

ompl::base::StateSamplerPtr PlanarManipulatorStateSpace::allocDefaultStateSampler() const
{
    return ompl::base::StateSamplerPtr(new PlanarManipulatorStateSampler(this));
}

ompl::base::State *PlanarManipulatorStateSpace::allocState() const
{
    StateType *rstate = new StateType();
    rstate->values = new double[dimension_];
    return rstate;
}

void PlanarManipulatorStateSpace::freeState(ompl::base::State *state) const
{
    StateType *rstate = static_cast<StateType *>(state);
    delete[] rstate->values;
    delete rstate;
}

void PlanarManipulatorStateSpace::registerProjections()
{
    // compute a default random projection
    if (dimension_ > 0)
        registerDefaultProjection(
            ompl::base::ProjectionEvaluatorPtr(new PlanarManipulatorProjectionEvaluator(this, manip_)));
}

double *PlanarManipulatorStateSpace::getValueAddressAtIndex(ompl::base::State *state, const unsigned int index) const
{
    return index < dimension_ ? static_cast<StateType *>(state)->values + index : nullptr;
}

unsigned int PlanarManipulatorStateSpace::getSerializationLength() const
{
    return dimension_ * sizeof(double);
}

void PlanarManipulatorStateSpace::serialize(void *serialization, const ompl::base::State *state) const
{
    memcpy(serialization, state->as<StateType>()->values, dimension_ * sizeof(double));
}

void PlanarManipulatorStateSpace::deserialize(ompl::base::State *state, const void *serialization) const
{
    memcpy(state->as<StateType>()->values, serialization, dimension_ * sizeof(double));
}

void PlanarManipulatorStateSpace::printState(const ompl::base::State *state, std::ostream &out) const
{
    out << "PlanarManipulatorState [";
    if (state)
    {
        const StateType *rstate = static_cast<const StateType *>(state);
        for (unsigned int i = 0; i < dimension_; ++i)
        {
            out << rstate->values[i];
            if (i + 1 < dimension_)
                out << ' ';
        }
    }
    else
        out << "nullptr" << std::endl;
    out << ']' << std::endl;
}

void PlanarManipulatorStateSpace::printSettings(std::ostream &out) const
{
    out << "Planar manipulator state space '" << getName() << "' of dimension " << dimension_ << std::endl;
}

void PlanarManipulatorStateSpace::linearInterpolate(const double &from, const double &to, const double &t,
                                                    double &out) const
{
    double diff = to - from;
    if (fabs(diff) <= PI)
        out = from + diff * t;
    else
    {
        if (diff > 0.0)
            diff = TWOPI - diff;
        else
            diff = -TWOPI - diff;

        out = from - diff * t;

        // input states are within bounds, so the following check is sufficient
        if (out > PI)
            out -= TWOPI;
        else if (out < -PI)
            out += TWOPI;
    }
}