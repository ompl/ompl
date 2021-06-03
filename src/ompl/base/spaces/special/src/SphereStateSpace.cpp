/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021,
 *  Max Planck Institute for Intelligent Systems (MPI-IS).
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
 *   * Neither the name of the MPI-IS nor the names
 *     of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written
 *     permission.
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

/* Author: Andreas Orthey */

#include <ompl/base/spaces/special/SphereStateSpace.h>
#include <ompl/tools/config/MagicConstants.h>
#include <cstring>
#include <cmath>
#include <boost/math/constants/constants.hpp>

using namespace boost::math::double_constants;  // pi
using namespace ompl::base;

SphereStateSampler::SphereStateSampler(const StateSpace *space) : StateSampler(space)
{
}

void SphereStateSampler::sampleUniform(State *state)
{
    // see for example http://corysimon.github.io/articles/uniformdistn-on-sphere/
    const double theta = 2.0 * pi * rng_.uniformReal(0, 1) - pi;  // uniform in [-pi,+pi]
    const double phi = acos(1.0 - 2.0 * rng_.uniformReal(0, 1));  // in [0,+pi]
    auto *S = state->as<SphereStateSpace::StateType>();
    S->setThetaPhi(theta, phi);
}

void SphereStateSampler::sampleUniformNear(State *state, const State *near, double distance)
{
    auto *S = state->as<SphereStateSpace::StateType>();
    const auto *Snear = near->as<SphereStateSpace::StateType>();
    S->setTheta(rng_.uniformReal(Snear->getTheta() - distance, Snear->getTheta() + distance));
    S->setPhi(rng_.uniformReal(Snear->getPhi() - distance, Snear->getPhi() + distance));
    space_->enforceBounds(state);
}

void SphereStateSampler::sampleGaussian(State *state, const State *mean, double stdDev)
{
    auto *S = state->as<SphereStateSpace::StateType>();
    const auto *Smean = mean->as<SphereStateSpace::StateType>();
    S->setTheta(rng_.gaussian(Smean->getTheta(), stdDev));
    S->setPhi(rng_.gaussian(Smean->getPhi(), stdDev));
    space_->enforceBounds(state);
}

StateSamplerPtr SphereStateSpace::allocDefaultStateSampler() const
{
    return std::make_shared<SphereStateSampler>(this);
}

SphereStateSpace::SphereStateSpace(double radius) : radius_(radius)
{
    setName("Sphere" + getName());
    type_ = STATE_SPACE_SPHERE;

    StateSpacePtr SO2(std::make_shared<SO2StateSpace>());
    StateSpacePtr R1(std::make_shared<RealVectorStateSpace>(1));
    R1->as<RealVectorStateSpace>()->setBounds(0, pi);

    addSubspace(SO2, 1.0);
    addSubspace(R1, 1.0);
    lock();
}

double SphereStateSpace::distance(const State *state1, const State *state2) const
{
    // https://en.wikipedia.org/wiki/Great-circle_distance#Formulae

    const auto *S1 = state1->as<SphereStateSpace::StateType>();
    const auto *S2 = state2->as<SphereStateSpace::StateType>();

    // Note: Formula assumes phi in [-pi/2,+pi/2]
    const float t1 = S1->getTheta();
    const float phi1 = S1->getPhi() - pi / 2.0;

    const float t2 = S2->getTheta();
    const float phi2 = S2->getPhi() - pi / 2.0;

    // This is the Vincenty formula, but it is less numerically stable
    // double dt = t2 - t1;
    // double d1 = std::pow(cos(phi2) * std::sin(dt), 2);
    // double d2 = std::pow(cos(phi1) * std::sin(phi2) - std::sin(phi1) * cos(phi2) * cos(dt), 2);
    // double numerator = std::sqrt(d1 + d2);
    // double denumerator = std::sin(phi1) * std::sin(phi2) + cos(phi1) * cos(phi2) * cos(dt);
    // return radius_ * atan2(numerator, denumerator);

    // Haversine formula
    const float s = 0.5 * (phi1 - phi2);
    const float t = 0.5 * (t1 - t2);
    const float d = std::sqrt(std::sin(s) * std::sin(s) + std::cos(phi1) * std::cos(phi2) * std::sin(t) * std::sin(t));
    return 2 * radius_ * asin(d);
}

double SphereStateSpace::getMeasure() const
{
    return 4 * pi * radius_ * radius_;
}

State *SphereStateSpace::allocState() const
{
    auto *state = new StateType();
    allocStateComponents(state);
    return state;
}

Eigen::Vector3f SphereStateSpace::toVector(const State *state) const
{
    Eigen::Vector3f v;

    const auto *S1 = state->as<SphereStateSpace::StateType>();
    const float theta = S1->getTheta();
    const float phi = S1->getPhi();

    v[0] = radius_ * std::sin(phi) * std::cos(theta);
    v[1] = radius_ * std::sin(phi) * std::sin(theta);
    v[2] = radius_ * std::cos(phi);

    return v;
}
