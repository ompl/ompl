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

#include <ompl/base/spaces/SphereStateSpace.h>
#include <ompl/tools/config/MagicConstants.h>
#include <cstring>
#include <boost/math/constants/constants.hpp>

using namespace boost::math::double_constants;  // pi
using namespace ompl::base;

SphereStateSampler::SphereStateSampler(const StateSpace *space) : StateSampler(space)
{
}

void SphereStateSampler::sampleUniform(State *state)
{
    // see for example http://corysimon.github.io/articles/uniformdistn-on-sphere/
    double theta = 2.0 * pi * rng_.uniformReal(0, 1) - pi;
    double phi = acos(1.0 - 2.0 * rng_.uniformReal(0, 1));
    SphereStateSpace::StateType *S = state->as<SphereStateSpace::StateType>();
    S->setThetaPhi(theta, phi);
}

void SphereStateSampler::sampleUniformNear(State *state, const State *near, double distance)
{
    SphereStateSpace::StateType *S = state->as<SphereStateSpace::StateType>();
    const SphereStateSpace::StateType *Snear = near->as<SphereStateSpace::StateType>();
    S->setTheta(rng_.uniformReal(Snear->getTheta() - distance, Snear->getTheta() + distance));
    S->setPhi(rng_.uniformReal(Snear->getPhi() - distance, Snear->getPhi() + distance));
    space_->enforceBounds(state);
}

void SphereStateSampler::sampleGaussian(State *state, const State *mean, double stdDev)
{
    SphereStateSpace::StateType *S = state->as<SphereStateSpace::StateType>();
    const SphereStateSpace::StateType *Smean = mean->as<SphereStateSpace::StateType>();
    S->setTheta(rng_.gaussian(Smean->getTheta(), stdDev));
    S->setPhi(rng_.gaussian(Smean->getPhi(), stdDev));
    space_->enforceBounds(state);
}

StateSamplerPtr SphereStateSpace::allocDefaultStateSampler() const
{
    return std::make_shared<SphereStateSampler>(this);
}

ompl::base::SphereStateSpace::SphereStateSpace()
{
    setName("Sphere" + getName());
    type_ = STATE_SPACE_UNKNOWN;
    addSubspace(std::make_shared<SO2StateSpace>(), 1.0);
    addSubspace(std::make_shared<SO2StateSpace>(), 1.0);
    lock();
}

double SphereStateSpace::distance(const State *state1, const State *state2) const
{
    // https://en.wikipedia.org/wiki/Great-circle_distance
    // https://en.wikipedia.org/wiki/Vincenty%27s_formulae

    const SphereStateSpace::StateType *S1 = state1->as<SphereStateSpace::StateType>();
    const SphereStateSpace::StateType *S2 = state2->as<SphereStateSpace::StateType>();

    double t1 = S1->getTheta();
    double p1 = S1->getPhi();

    double t2 = S2->getTheta();
    double p2 = S2->getPhi();

    double dt = t2 - t1;
    double d1 = powf(cos(p2) * sin(dt), 2);
    double d2 = powf(cos(p1) * sin(p2) - sin(p1) * cos(p2) * cos(dt), 2);

    double numerator = sqrtf(d1 + d2);

    double denumerator = sin(p1) * sin(p2) + cos(p1) * cos(p2) * cos(dt);

    return radius_ * atan2(numerator, denumerator);
}

double SphereStateSpace::getMeasure() const
{
    return 4 * pi * radius_ * radius_;
}

ompl::base::State *ompl::base::SphereStateSpace::allocState() const
{
    auto *state = new StateType();
    allocStateComponents(state);
    return state;
}
