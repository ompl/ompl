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

#include <ompl/base/spaces/special/MobiusStateSpace.h>
#include <ompl/tools/config/MagicConstants.h>
#include <cstring>
#include <cmath>
#include <boost/math/constants/constants.hpp>

using namespace boost::math::double_constants;  // pi
using namespace ompl::base;

MobiusStateSpace::MobiusStateSpace(double intervalMax, double radius) : intervalMax_(intervalMax), radius_(radius)
{
    setName("Mobius" + getName());
    type_ = STATE_SPACE_MOBIUS;

    StateSpacePtr SO2(std::make_shared<SO2StateSpace>());
    StateSpacePtr R1(std::make_shared<RealVectorStateSpace>(1));
    R1->as<RealVectorStateSpace>()->setBounds(-intervalMax, +intervalMax);

    addSubspace(SO2, 1.0);
    addSubspace(R1, 1.0);
    lock();
}

double MobiusStateSpace::distance(const State *state1, const State *state2) const
{
    double theta1 = state1->as<MobiusStateSpace::StateType>()->getU();
    double theta2 = state2->as<MobiusStateSpace::StateType>()->getU();

    double diff = theta2 - theta1;

    if (std::abs(diff) <= pi)
    {
        return CompoundStateSpace::distance(state1, state2);
    }
    else
    {
        // requires interpolation over the gluing strip
        const auto *cstate1 = static_cast<const CompoundState *>(state1);
        const auto *cstate2 = static_cast<const CompoundState *>(state2);

        // distance on S1 as usual
        double dist = 0.0;
        dist += weights_[0] * components_[0]->distance(cstate1->components[0], cstate2->components[0]);

        double r1 = state1->as<MobiusStateSpace::StateType>()->getV();
        double r2 = state2->as<MobiusStateSpace::StateType>()->getV();

        r2 = -r2;

        dist += std::sqrt((r2 - r1) * (r2 - r1));
        return dist;
    }
}

void MobiusStateSpace::interpolate(const State *from, const State *to, double t, State *state) const
{
    double theta1 = from->as<MobiusStateSpace::StateType>()->getU();
    double theta2 = to->as<MobiusStateSpace::StateType>()->getU();

    double diff = theta2 - theta1;

    if (std::abs(diff) <= pi)
    {
        // interpolate as it would be a cylinder
        CompoundStateSpace::interpolate(from, to, t, state);
    }
    else
    {
        // requires interpolation over the gluing strip
        const auto *cfrom = static_cast<const CompoundState *>(from);
        const auto *cto = static_cast<const CompoundState *>(to);
        auto *cstate = static_cast<CompoundState *>(state);

        // interpolate S1 as usual
        components_[0]->interpolate(cfrom->components[0], cto->components[0], t, cstate->components[0]);

        double r1 = from->as<MobiusStateSpace::StateType>()->getV();
        double r2 = to->as<MobiusStateSpace::StateType>()->getV();

        // Need to mirror point for interpolation
        r2 = -r2;

        double r = r1 + (r2 - r1) * t;

        // check again if we need to invert (only if we already crossed gluing
        // line)
        double thetaNew = state->as<MobiusStateSpace::StateType>()->getU();
        double diff2 = theta2 - thetaNew;

        if (std::abs(diff2) <= pi)
        {
            r = -r;
        }

        state->as<MobiusStateSpace::StateType>()->setV(r);
    }
}

State *MobiusStateSpace::allocState() const
{
    auto *state = new StateType();
    allocStateComponents(state);
    return state;
}

Eigen::Vector3f MobiusStateSpace::toVector(const State *state) const
{
    Eigen::Vector3f vec;

    const auto *s = state->as<MobiusStateSpace::StateType>();
    float u = s->getU();
    float v = s->getV();

    double R = radius_ + v * std::cos(0.5 * u);
    vec[0] = R * std::cos(u);
    vec[1] = R * std::sin(u);
    vec[2] = v * std::sin(0.5 * u);
    return vec;
}
