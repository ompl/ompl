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
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>

namespace ob = ompl::base;

KoulesStateSpace::KoulesStateSpace(unsigned int numKoules)
    : CompoundStateSpace(), mass_(numKoules + 1, kouleMass), radius_(numKoules + 1, kouleRadius)
{
    mass_[numKoules] = shipMass;
    radius_[numKoules] = shipRadius;
    setName("Koules" + boost::lexical_cast<std::string>(numKoules) + getName());
    // layout: (... x_i y_i vx_i vy_i ... x_s y_s vx_s vy_s theta_s),
    // where (x_i, y_i) is the position of koule i (i=1,..,numKoules),
    // (vx_i, vy_i) its velocity, (x_s, y_s) the position of the ship,
    // (vx_s, vy_s) its velocity, and theta_s its orientation.
    addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(4 * (numKoules + 1))), 1.);
    addSubspace(ob::StateSpacePtr(new ob::SO2StateSpace), .5);
    lock();

    // create the bounds
    ob::RealVectorBounds bounds((numKoules + 1) * 4);
    unsigned int j = 0;
    for (unsigned int i = 0; i < numKoules; ++i)
    {
        // set the bounds for koule i's position
        bounds.setLow(j, -kouleRadius);
        bounds.setHigh(j++, sideLength + kouleRadius);
        bounds.setLow(j, -kouleRadius);
        bounds.setHigh(j++, sideLength + kouleRadius);
        // set the bounds for koule i's velocity
        bounds.setLow(j, -10);
        bounds.setHigh(j++, 10.);
        bounds.setLow(j, -10.);
        bounds.setHigh(j++, 10.);
    }
    // set the bounds for the ship's position
    bounds.setLow(j, shipRadius);
    bounds.setHigh(j++, sideLength - shipRadius);
    bounds.setLow(j, shipRadius);
    bounds.setHigh(j++, sideLength - shipRadius);
    // set the bounds for the ship's velocity
    bounds.setLow(j, -10.);
    bounds.setHigh(j++, 10.);
    bounds.setLow(j, -10.);
    bounds.setHigh(j++, 10.);
    as<ob::RealVectorStateSpace>(0)->setBounds(bounds);
}

void KoulesStateSpace::registerProjections(void)
{
    registerDefaultProjection(ob::ProjectionEvaluatorPtr(new KoulesProjection(this)));
    registerProjection("PDSTProjection", ob::ProjectionEvaluatorPtr(
        new KoulesProjection(this, (getDimension() - 1) / 2 + 1)));
}
