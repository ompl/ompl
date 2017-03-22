/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Rice University
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

/* Author: Beck Chen, Mark Moll */

#include "KoulesConfig.h"
#include "KoulesStateSpace.h"
#include "KoulesControlSpace.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace ob = ompl::base;
namespace oc = ompl::control;

void KoulesControlSampler::sample(oc::Control *control)
{
    const ob::RealVectorBounds &bounds = space_->as<oc::RealVectorControlSpace>()->getBounds();
    auto *rcontrol =
        control->as<oc::RealVectorControlSpace::ControlType>();
    double r = rng_.uniformReal(bounds.low[0], bounds.high[0]);
    double theta = rng_.uniformReal(0., 2. * boost::math::constants::pi<double>());
    rcontrol->values[0] = r * cos(theta);
    rcontrol->values[1] = r * sin(theta);
}

void KoulesControlSampler::sample(ompl::control::Control *control, const ompl::base::State *state)
{
    steer(control, state, rng_.uniformReal(0., sideLength), rng_.uniformReal(0., sideLength));
}

void KoulesControlSampler::steer(oc::Control *control, const ob::State *state, double x, double y)
{
    const double* r = state->as<KoulesStateSpace::StateType>()->values;
    double dx = x - r[0];
    double dy = y - r[1];
    double xNrm2 = dx * dx + dy * dy;
    if (xNrm2 > std::numeric_limits<float>::epsilon())
    {
        const ob::RealVectorBounds &bounds = space_->as<oc::RealVectorControlSpace>()->getBounds();
        double v = rng_.uniformReal(bounds.low[0], bounds.high[0]) / sqrt(xNrm2);
        auto *rcontrol =
            control->as<oc::RealVectorControlSpace::ControlType>();
        rcontrol->values[0] = v * dx;
        rcontrol->values[1] = v * dy;
    }
    else
        sample(control);
}

KoulesControlSpace::KoulesControlSpace(unsigned int numKoules)
    : ompl::control::RealVectorControlSpace(
        std::make_shared<KoulesStateSpace>(numKoules), 2)
{
    bounds_.setLow(shipVmin);
    bounds_.setHigh(shipVmax);
}

