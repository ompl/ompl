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

#include "KoulesStateSpace.h"
#include "KoulesControlSpace.h"
#include "KoulesStatePropagator.h"
#include <ompl/base/spaces/SO2StateSpace.h>

namespace ob = ompl::base;
namespace oc = ompl::control;

// lightweight signed SO(2) distance; assumes x and y are in [-pi,pi]
double signedSO2Distance(double x, double y)
{
    double d0 = x - y;
    if (d0 < -boost::math::constants::pi<double>())
        return d0 + 2. * boost::math::constants::pi<double>();
    if (d0 > boost::math::constants::pi<double>())
        return d0 - 2. * boost::math::constants::pi<double>();
    return d0;
}


void KoulesStatePropagator::propagate(const ob::State *start, const oc::Control* control,
    const double duration, ob::State *result) const
{
    const double* cval = control->as<KoulesControlSpace::ControlType>()->values;
    unsigned int numSteps = ceil(duration / timeStep_), offset = 4 * numKoules_;
    double dt = duration / (double)numSteps, u[3] = {0., 0., 0.};

    si_->getStateSpace()->copyToReals(q, start);

    double v[2] = { cval[0] - q[offset + 2], cval[1] - q[offset + 3]};
    double deltaTheta = signedSO2Distance(atan2(v[1], v[0]), q[offset + 4]);
    if (v[0]*v[0] + v[1]*v[1] > shipDelta * shipDelta)
    {
        if (std::abs(deltaTheta) < shipEps)
        {
            u[0] = shipAcceleration * cos(q[offset + 4]);
            u[1] = shipAcceleration * sin(q[offset + 4]);
            u[2] = 0.;
        }
        else if (deltaTheta > 0)
            u[2] = shipRotVel;
        else
            u[2] = -shipRotVel;
    }
    for (unsigned int i = 0; i < numSteps; ++i)
    {
        ode(u);
        update(dt);
    }
    si_->getStateSpace()->copyFromReals(result, q);
    // Normalize orientation between -pi and pi
    si_->getStateSpace()->as<ob::CompoundStateSpace>()->as<ob::SO2StateSpace>(1)
        ->enforceBounds(result->as<ob::CompoundStateSpace::StateType>()
            ->as<ob::SO2StateSpace::StateType>(1));
}

void KoulesStatePropagator::ode(double* u) const
{
    // koules: qdot[4*i, 4*i + 1] is xdot, qdot[4*i + 2, 4*i + 3] is vdot
    unsigned int offset = 4 * numKoules_;
    for (unsigned int i = 0; i < offset; i += 4)
    {
        qdot[i    ] = q[i + 2];
        qdot[i + 1] = q[i + 3];
        qdot[i + 2] = (.5 * sideLength - q[i    ]) * lambda_c - q[i + 2] * h;
        qdot[i + 3] = (.5 * sideLength - q[i + 1]) * lambda_c - q[i + 3] * h;
    }
    // ship: qdot[offset, offset + 1] is xdot
    // ship: qdot[offset + 4] + ] is thetadot, qdot[offset + 2, offset + 3] is vdot
    qdot[offset    ] = q[offset + 2];
    qdot[offset + 1] = q[offset + 3];
    qdot[offset + 2] = u[0];
    qdot[offset + 3] = u[1];
    qdot[offset + 4] = u[2];
}

void KoulesStatePropagator::update(double dt) const
{
    // update collisions
    std::fill(hasCollision.begin(), hasCollision.end(), false);
    for (unsigned int i = 0; i < numKoules_; i++)
        for (unsigned int j = i + 1; j <= numKoules_; j++)
            if (checkCollision(i, j, dt))
                hasCollision[i] = hasCollision[j] = true;

    // update objects with no collision according to qdot
    for (unsigned int i = 0; i < numKoules_; ++i)
        if (!hasCollision[i])
            for (unsigned int j = 0; j < 4; ++j)
                q[4 * i + j] += qdot[4 * i + j] * dt;
    if (!hasCollision[numKoules_])
        for (unsigned int j = 0; j < 5; ++j)
            q[4 * numKoules_ + j] += qdot[4 * numKoules_ + j] * dt;
}

// check collision among object i and j
// compute elastic collision response if i and j collide
// see http://en.wikipedia.org/wiki/Elastic_collision
bool KoulesStatePropagator::checkCollision(unsigned int i, unsigned int j, double dt) const
{
    static const float delta = 1e-5;
    double *a = &q[4 * i], *b = &q[4 * j];
    double dx = a[0] - b[0], dy = a[1] - b[1];
    double dist = dx * dx + dy * dy;
    double minDist = si_->getStateSpace()->as<KoulesStateSpace>()->getRadius(i) +
        si_->getStateSpace()->as<KoulesStateSpace>()->getRadius(j) + delta;
    if (dist < minDist*minDist && ((b[2] - a[2]) * dx + (b[3] - a[3]) * dy > 0))
    // close enough and moving closer; elastic collision happens
    {
        dist = std::sqrt(dist);
        // compute unit normal and tangent vectors
        double normal[2] = {dx / dist, dy / dist};
        double tangent[2] = {-normal[1], normal[0]};

        // compute scalar projections of velocities onto normal and tangent vectors
        double aNormal = normal[0] * a[2] + normal[1] * a[3];
        double aTangentPrime = tangent[0] * a[2] + tangent[1] * a[3];
        double bNormal = normal[0] * b[2] + normal[1] * b[3];
        double bTangentPrime = tangent[0] * b[2] + tangent[1] * b[3];

        // compute new velocities using one-dimensional elastic collision in the normal direction
        double massA = si_->getStateSpace()->as<KoulesStateSpace>()->getMass(i);
        double massB = si_->getStateSpace()->as<KoulesStateSpace>()->getMass(j);
        double aNormalPrime = (aNormal * (massA - massB) + 2. * massB * bNormal) / (massA + massB);
        double bNormalPrime = (bNormal * (massB - massA) + 2. * massA * aNormal) / (massA + massB);

        // compute new normal and tangential velocity vectors
        double aNewNormalVel[2] = {normal[0] * aNormalPrime, normal[1] * aNormalPrime};
        double aNewTangentVel[2] = {tangent[0] * aTangentPrime, tangent[1] * aTangentPrime};
        double bNewNormalVel[2] = {normal[0] * bNormalPrime, normal[1] * bNormalPrime};
        double bNewTangentVel[2] = {tangent[0] * bTangentPrime, tangent[1] * bTangentPrime};

        // compute new velocities
        double bNewVel[2] = { bNewNormalVel[0] + bNewTangentVel[0], bNewNormalVel[1] + bNewTangentVel[1] };
        double aNewVel[2] = { aNewNormalVel[0] + aNewTangentVel[0], aNewNormalVel[1] + aNewTangentVel[1] };

        // preservation of momemtum
        assert(std::abs(massA * (a[2]-aNewVel[0]) + massB * (b[2]-bNewVel[0])) < 1e-6);
        assert(std::abs(massA * (a[3]-aNewVel[1]) + massB * (b[3]-bNewVel[1])) < 1e-6);
        // preservation of kinetic energy
        assert(std::abs(massA * (a[2]*a[2] + a[3]*a[3] - aNewVel[0]*aNewVel[0] - aNewVel[1]*aNewVel[1])
            + massB * (b[2]*b[2] + b[3]*b[3] - bNewVel[0]*bNewVel[0] - bNewVel[1]*bNewVel[1])) < 1e-6);

        // update state if collision happens
        a[0] += aNewVel[0] * dt;
        a[1] += aNewVel[1] * dt;
        a[2] = aNewVel[0];
        a[3] = aNewVel[1];
        b[0] += bNewVel[0] * dt;
        b[1] += bNewVel[1] * dt;
        b[2] = bNewVel[0];
        b[3] = bNewVel[1];

        return true;
    }
    else
        return false;
}
