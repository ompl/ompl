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

#include <ompl/base/spaces/special/TorusStateSpace.h>
#include <ompl/tools/config/MagicConstants.h>
#include <cstring>
#include <cmath>

#include <boost/math/constants/constants.hpp>
using namespace boost::math::double_constants;  // pi
using namespace ompl::base;

TorusStateSampler::TorusStateSampler(const StateSpace *space) : StateSampler(space)
{
}

void TorusStateSampler::sampleUniform(State *state)
{
    // https://stackoverflow.com/questions/26300510/generating-random-points-on-a-surface-of-an-n-dimensional-torus
    // Based on publication "Random selection of points distributed on curved surfaces."
    // Link: https://iopscience.iop.org/article/10.1088/0031-9155/32/10/009/pdf
    const auto *T = static_cast<const TorusStateSpace *>(space_);

    bool acceptedSampleFound = false;
    while (!acceptedSampleFound)
    {
        const double u = rng_.uniformReal(-pi, pi);
        const double v = rng_.uniformReal(-pi, pi);

        const double &R = T->getMajorRadius();
        const double &r = T->getMinorRadius();

        const double vprime = (R + r * std::cos(v)) / (R + r);

        const double mu = rng_.uniformReal(0, 1);
        if (mu <= vprime)
        {
            auto *T = state->as<TorusStateSpace::StateType>();
            T->setS1S2(u, v);
            acceptedSampleFound = true;
        }
    }
}

void TorusStateSampler::sampleUniformNear(State *state, const State *near, double distance)
{
    auto *T = state->as<TorusStateSpace::StateType>();
    const auto *Tnear = near->as<TorusStateSpace::StateType>();
    T->setS1(rng_.uniformReal(Tnear->getS1() - distance, Tnear->getS1() + distance));
    T->setS2(rng_.uniformReal(Tnear->getS2() - distance, Tnear->getS2() + distance));
    space_->enforceBounds(state);
}

void TorusStateSampler::sampleGaussian(State *state, const State *mean, double stdDev)
{
    auto *T = state->as<TorusStateSpace::StateType>();
    const auto *Tmean = mean->as<TorusStateSpace::StateType>();
    T->setS1(rng_.gaussian(Tmean->getS1(), stdDev));
    T->setS2(rng_.gaussian(Tmean->getS2(), stdDev));

    space_->enforceBounds(state);
}

TorusStateSpace::TorusStateSpace(double majorRadius, double minorRadius)
  : majorRadius_(majorRadius), minorRadius_(minorRadius)
{
    setName("Torus" + getName());
    type_ = STATE_SPACE_TORUS;
    addSubspace(std::make_shared<SO2StateSpace>(), 1.0);
    addSubspace(std::make_shared<SO2StateSpace>(), 1.0);
    lock();
}

StateSamplerPtr TorusStateSpace::allocDefaultStateSampler() const
{
    return std::make_shared<TorusStateSampler>(this);
}

double TorusStateSpace::distance(const State *state1, const State *state2) const
{
    const auto *cstate1 = static_cast<const CompoundState *>(state1);
    const auto *cstate2 = static_cast<const CompoundState *>(state2);
    const double x = components_[0]->distance(cstate1->components[0], cstate2->components[0]);
    const double y = components_[1]->distance(cstate1->components[1], cstate2->components[1]);
    return std::sqrt(x * x + y * y);
}

State *TorusStateSpace::allocState() const
{
    auto *state = new StateType();
    allocStateComponents(state);
    return state;
}

double TorusStateSpace::getMajorRadius() const
{
    return majorRadius_;
}

double TorusStateSpace::getMinorRadius() const
{
    return minorRadius_;
}

Eigen::Vector3f TorusStateSpace::toVector(const State *state) const
{
    Eigen::Vector3f v;

    const auto *s = state->as<TorusStateSpace::StateType>();
    const float theta = s->getS1();
    const float phi = s->getS2();

    const double &R = majorRadius_;
    const double &r = minorRadius_;

    v[0] = (R + r * std::cos(phi)) * std::cos(theta);
    v[1] = (R + r * std::cos(phi)) * std::sin(theta);
    v[2] = r * std::sin(phi);

    return v;
}
