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

#include "KoulesConfig.h"
#include "KoulesGoal.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>

double KoulesGoal::distanceGoal(const ompl::base::State *st) const
{
    // the shortest distance between any koule and an edge
    double minDist = sideLength;
    double minX, minY;
    const double* v = st->as<ompl::base::CompoundStateSpace::StateType>()
        ->as<ompl::base::RealVectorStateSpace::StateType>(0)->values;
    for (unsigned int i = 0; i < numKoules_; ++i)
    {
        minX = std::min(v[4 * i    ], sideLength - v[4 * i    ]);
        minY = std::min(v[4 * i + 1], sideLength - v[4 * i + 1]);
        minDist = std::min(minDist, std::min(minX, minY) - kouleRadius + threshold_);
    }
    if (minDist < 0)
        minDist = 0;
    return minDist;
}

void KoulesGoal::sampleGoal(ompl::base::State *st) const
{
    double* v = st->as<ompl::base::CompoundStateSpace::StateType>()
        ->as<ompl::base::RealVectorStateSpace::StateType>(0)->values;
    stateSampler_->sampleUniform(st);
    for (unsigned i = 0; i < numKoules_; ++i)
    {
        // randomly pick an edge for each koule to collide
        if (rng_.uniformBool())
        {
            v[4 * i    ] = rng_.uniformBool() ? 0. : sideLength;
            v[4 * i + 1] = rng_.uniformReal(0., sideLength);
        }
        else
        {
            v[4 * i    ] = rng_.uniformReal(0., sideLength);
            v[4 * i + 1] = rng_.uniformBool() ? 0. : sideLength;
        }
    }
}
