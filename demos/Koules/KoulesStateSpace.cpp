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
#include "KoulesProjection.h"

namespace ob = ompl::base;

KoulesStateSpace::KoulesStateSpace(unsigned int numKoules)
    : RealVectorStateSpace(4 * numKoules + 5), mass_(numKoules + 1, kouleMass),
    radius_(numKoules + 1, kouleRadius)
{
    mass_[0] = shipMass;
    radius_[0] = shipRadius;
    setName("Koules" + std::to_string(numKoules) + getName());
    // layout: (x_s y_s vx_s vy_s theta_s ... x_i y_i vx_i vy_i ... ),
    // where (x_i, y_i) is the position of koule i (i=1,..,numKoules),
    // (vx_i, vy_i) its velocity, (x_s, y_s) the position of the ship,
    // (vx_s, vy_s) its velocity, and theta_s its orientation.

    // create the bounds
    unsigned int j = 0;
    // set the bounds for the ship's position
    bounds_.setLow(j, shipRadius);
    bounds_.setHigh(j++, sideLength - shipRadius);
    bounds_.setLow(j, shipRadius);
    bounds_.setHigh(j++, sideLength - shipRadius);
    // set the bounds for the ship's velocity
    bounds_.setLow(j, -10.);
    bounds_.setHigh(j++, 10.);
    bounds_.setLow(j, -10.);
    bounds_.setHigh(j++, 10.);
    // set bounds on orientation
    bounds_.setLow(j, -boost::math::constants::pi<double>());
    bounds_.setHigh(j++, boost::math::constants::pi<double>());
    for (unsigned int i = 0; i < numKoules; ++i)
    {
        // set the bounds for koule i's position
        bounds_.setLow(j, -2. * kouleRadius);
        bounds_.setHigh(j++, sideLength + 2. * kouleRadius);
        bounds_.setLow(j, -2. * kouleRadius);
        bounds_.setHigh(j++, sideLength + 2. * kouleRadius);
        // set the bounds for koule i's velocity
        bounds_.setLow(j, -10);
        bounds_.setHigh(j++, 10.);
        bounds_.setLow(j, -10.);
        bounds_.setHigh(j++, 10.);
    }
}

void KoulesStateSpace::registerProjections()
{
    registerDefaultProjection(std::make_shared<KoulesProjection>(this, 3));
    registerProjection("PDSTProjection", std::make_shared<KoulesProjection>(this, (getDimension() - 1) / 2 + 1));
}

bool KoulesStateSpace::isDead(const ompl::base::State* state, unsigned int i) const
{
    const auto* s = static_cast<const StateType*>(state);
    return s->values[i != 0u ? 4 * i + 1 : 0] == -2. * kouleRadius;
}
