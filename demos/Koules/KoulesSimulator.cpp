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

#include <ompl/control/SpaceInformation.h>
#include "KoulesStateSpace.h"
#include "KoulesControlSpace.h"
#include "KoulesSimulator.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

namespace
{
    inline double normalizeAngle(double theta)
    {
        if (theta < -boost::math::constants::pi<double>())
            return theta + 2. * boost::math::constants::pi<double>();
        if (theta > boost::math::constants::pi<double>())
            return theta - 2. * boost::math::constants::pi<double>();
        return theta;
    }

    inline void normalize2(double* v)
    {
        double nrm = std::sqrt(v[0] * v[0] + v[1] * v[1]);
        if (nrm > std::numeric_limits<double>::epsilon())
        {
            v[0] /= nrm;
            v[1] /= nrm;
        }
        else
        {
            v[0] = 1.;
            v[1] = 0.;
        }
    }
    inline double dot2(const double* v, const double* w)
    {
        return v[0] * w[0] + v[1] * w[1];
    }
    inline void vadd2(double* v, double s, const double* w)
    {
        v[0] += s * w[0];
        v[1] += s * w[1];
    }
    inline unsigned int ind(unsigned int i)
    {
        return i != 0u ? 4 * i + 1 : 0;
    }
    inline void ode(const double* y, double* dydx)
    {
        dydx[0] = y[2];
        dydx[1] = y[3];
        dydx[2] = (.5 * sideLength - y[0]) * lambda_c - y[2] * h;
        dydx[3] = (.5 * sideLength - y[1]) * lambda_c - y[3] * h;
    }
    inline void rungeKutta4(double* y, double h, double* yout)
    {
        double hh = h * .5, h6 = h / 6.;
        static double yt[4], dydx[4], dym[4], dyt[4];
        ode(y, dydx);
        for (unsigned int i = 0; i < 4; ++i)
            yt[i] = y[i] + hh * dydx[i];
        ode(yt, dyt);
        for (unsigned int i = 0; i < 4; ++i)
            yt[i] = y[i] + hh * dyt[i];
        ode(yt, dym);
        for (unsigned int i = 0; i < 4; ++i)
        {
            yt[i] = y[i] + h * dym[i];
            dym[i] += dyt[i];
        }
        ode(yt, dyt);
        for (unsigned int i = 0; i < 4; ++i)
            yout[i] = y[i] + h6 * (dydx[i] + dyt[i] + 2. * dym[i]);
    }

    const float eps = std::numeric_limits<float>::epsilon();
}

KoulesSimulator::KoulesSimulator(const ompl::control::SpaceInformation* si) :
    si_(si),
    numDimensions_(si->getStateSpace()->getDimension()),
    numKoules_((numDimensions_ - 5) / 4),
    qcur_(numDimensions_),
    qnext_(numDimensions_),
    dead_(numKoules_ + 1)
{
}

void KoulesSimulator::updateShip(const oc::Control* control, double t)
{
    const double* cval = control->as<KoulesControlSpace::ControlType>()->values;
    double v[2] = { cval[0] - qcur_[2], cval[1] - qcur_[3] };
    double deltaTheta = normalizeAngle(atan2(v[1], v[0]) - qcur_[4]);
    double accel = 0., omega = 0.;

    if (std::abs(deltaTheta) < shipEps)
    {
        if (v[0] * v[0] + v[1] * v[1] > shipDelta * shipDelta)
            accel = shipAcceleration;
    }
    else
        omega = deltaTheta > 0. ? shipRotVel : -shipRotVel;

    if (accel == 0.)
    {
        qnext_[0] = qcur_[0] + qcur_[2] * t;
        qnext_[1] = qcur_[1] + qcur_[3] * t;
        qnext_[2] = qcur_[2];
        qnext_[3] = qcur_[3];
        qcur_[4] = normalizeAngle(qcur_[4] + omega * t);
    }
    else // omega == 0.
    {
        double ax = accel * cos(qcur_[4]);
        double ay = accel * sin(qcur_[4]);
        double t2 = .5 * t * t;
        qnext_[0] = qcur_[0] + qcur_[2] * t + ax * t2;
        qnext_[1] = qcur_[1] + qcur_[3] * t + ay * t2;
        qnext_[2] = qcur_[2] + ax * t;
        qnext_[3] = qcur_[3] + ay * t;
    }
}

void KoulesSimulator::initCollisionEvents()
{
    double r[2], d, bad, ri, rij;
    unsigned int ii, jj;
    collisionEvents_ = CollisionEventQueue();
    for (unsigned int i = 0; i < numKoules_; ++i)
        if (!dead_[i])
        {
            ii = ind(i);
            ri = si_->getStateSpace()->as<KoulesStateSpace>()->getRadius(i);
            for (unsigned int j = i + 1; j <= numKoules_; ++j)
                if (!dead_[j])
                {
                    jj = ind(j);
                    rij = ri  + si_->getStateSpace()->as<KoulesStateSpace>()->getRadius(j);
                    r[0] = qcur_[jj    ] - qcur_[ii    ];
                    r[1] = qcur_[jj + 1] - qcur_[ii + 1];
                    d = r[0] * r[0] + r[1] * r[1];
                    if (d < rij * rij)
                    {
                        d = std::sqrt(d);
                        bad = rij - d;
                        r[0] /= d;
                        r[0] *= bad * (1. + eps) * .5;
                        r[1] /= d;
                        r[1] *= bad * (1. + eps) * .5;
                        qcur_[ii    ] -= r[0];
                        qcur_[ii + 1] -= r[1];
                        qcur_[jj    ] += r[0];
                        qcur_[jj + 1] += r[1];
                    }
                }
        }
    for (unsigned int i = 0; i <= numKoules_; ++i)
        for (unsigned int j = i + 1; j <= numKoules_ + 2; ++j)
            computeCollisionEvent(i, j);
}

double KoulesSimulator::wallCollideEvent(unsigned int i, int dim)
{
    double r = si_->getStateSpace()->as<KoulesStateSpace>()->getRadius(i);
    unsigned int ii = ind(i);
    if (qcur_[ii + 2 + dim] > 0.)
        return std::max(0., (sideLength - r - qcur_[ii + dim]) / qcur_[ii + 2 + dim]);
    if (qcur_[ii + 2 + dim] < 0.)
        return std::max(0., (r - qcur_[ii + dim]) / qcur_[ii + 2 + dim]);
    else
        return -1.;
}

void KoulesSimulator::computeCollisionEvent(unsigned int i, unsigned int j)
{
    if (dead_[i] || (j <= numKoules_ && dead_[j]))
        return;

    double ri = si_->getStateSpace()->as<KoulesStateSpace>()->getRadius(i);
    double t;

    if (j == numKoules_ + 1)
        t = wallCollideEvent(i, 0);
    else if (j == numKoules_ + 2)
        t = wallCollideEvent(i, 1);
    else
    {
        unsigned int ii = ind(i), jj = ind(j);
        double u[2] = { qcur_[ii + 2] - qcur_[jj + 2], qcur_[ii + 3] - qcur_[jj + 3] };
        double v[2] = { qcur_[ii    ] - qcur_[jj    ], qcur_[ii + 1] - qcur_[jj + 1] };
        double a = u[0] * u[0] + u[1] * u[1];
        if (a == 0.)
            t = -1.;
        else
        {
            double r = ri + si_->getStateSpace()->as<KoulesStateSpace>()->getRadius(j);
            double b = 2 * u[0] * v[0] + 2 * u[1] * v[1];
            double c = v[0] * v[0] + v[1] * v[1] - r * r;
            double disc = b * b - 4. * a * c;
            if (std::abs(disc) < std::numeric_limits<float>::epsilon())
                t = -.5 * b / a;
            else if (disc < 0.)
                t = -1.;
            else
            {
                disc = std::sqrt(disc);
                t = (-b - disc) / (2. * a);
                if (t < 0.)
                    t = (-b + disc) / (2. * a);
            }
        }
    }
    t += time_;
    if (t >= time_ && t <= endTime_)
        collisionEvents_.emplace(t, i, j);
}

void KoulesSimulator::elasticCollision(unsigned int i, unsigned int j)
{
    double *a = &qcur_[ind(i)], *b = &qcur_[ind(j)];
    double d[2] = { b[0] - a[0], b[1] - a[1] };
    normalize2(d);
    double va = dot2(a + 2, d), vb = dot2(b + 2, d);
    if (va - vb <= 0.)
        return;
    double ma = si_->getStateSpace()->as<KoulesStateSpace>()->getMass(i);
    double mb = si_->getStateSpace()->as<KoulesStateSpace>()->getMass(j);
    double vap = (va * (ma - mb) + 2. * mb * vb) / (ma + mb);
    double vbp = (vb * (mb - ma) + 2. * ma * va) / (ma + mb);
    double amag = vap - va, bmag = vbp - vb;
    if (std::abs(amag) < eps)
        amag = amag < 0. ? -eps : eps;
    if (std::abs(bmag) < eps)
        bmag = bmag < 0. ? -eps : eps;
#ifndef NDEBUG
    double px = ma * a[2] + mb * b[2], py = ma * a[3] + mb * b[3];
    double k = ma * (a[2] * a[2] + a[3] * a[3]) + mb * (b[2] * b[2] + b[3] * b[3]);
#endif
    vadd2(a + 2, amag, d);
    vadd2(b + 2, bmag, d);

#ifndef NDEBUG
    // preservation of momemtum
    assert(std::abs(ma * a[2] + mb * b[2] - px) < 1e-6);
    assert(std::abs(ma * a[3] + mb * b[3] - py) < 1e-6);
    // preservation of kinetic energy
    assert(std::abs(ma * (a[2] * a[2] + a[3] * a[3]) + mb * (b[2] * b[2] + b[3] * b[3]) - k) < 1e-6);
#endif
}

void KoulesSimulator::advance(double t)
{
    double dt = t - time_;
    qcur_[0] += qcur_[2] * dt;
    qcur_[1] += qcur_[3] * dt;
    for (unsigned int i = 5; i < numDimensions_; i += 4)
    {
        qcur_[i    ] += qcur_[i + 2] * dt;
        qcur_[i + 1] += qcur_[i + 3] * dt;
    }
    time_ = t;
}

void KoulesSimulator::markAsDead(unsigned int i)
{
    unsigned int ii = ind(i);
    qcur_[ii] = -2. * kouleRadius;
    qcur_[ii + 1] = qcur_[ii + 2] = qcur_[ii + 3] = 0.;
}

void KoulesSimulator::step(const ob::State *start, const oc::Control* control,
    const double t, ob::State *result)
{
    unsigned int ii;

    memcpy(&qcur_[0], start->as<KoulesStateSpace::StateType>()->values, numDimensions_ * sizeof(double));
    time_ = 0.;
    endTime_ = t;
    std::fill(dead_.begin(), dead_.end(), false);
    updateShip(control, t);
    for (unsigned int i = 0; i <= numKoules_; ++i)
    {
        ii = ind(i);
        dead_[i] = qcur_[ii] == -2. * kouleRadius;
        if (!dead_[i])
        {
            if (i != 0u)
                rungeKutta4(&qcur_[ii], t, &qnext_[ii]);
            qcur_[ii + 2] = (qnext_[ii    ] - qcur_[ii    ]) / t;
            qcur_[ii + 3] = (qnext_[ii + 1] - qcur_[ii + 1]) / t;
        }
    }
    initCollisionEvents();
    while (!collisionEvents_.empty())
    {
        CollisionEvent event = collisionEvents_.top();
        double ct = std::get<0>(event);
        unsigned int i = std::get<1>(event), j = std::get<2>(event);

        collisionEvents_.pop();
        advance(ct);
        if (j <= numKoules_)
            elasticCollision(i, j);
        else
        {
            markAsDead(i);
            if (i == 0)
            {
                memcpy(result->as<KoulesStateSpace::StateType>()->values, &qcur_[0], numDimensions_ * sizeof(double));
                return;
            }
        }

        for (unsigned int k = 0; k <= numKoules_ + 2; ++k)
        {
            if (k < i)
                computeCollisionEvent(k, i);
            else if (k > i && k != j)
                computeCollisionEvent(i, k);
            if (k < j && k != i)
                computeCollisionEvent(k, j);
            else if (k > j)
                computeCollisionEvent(j, k);
        }
    }
    advance(t);
    memcpy(result->as<KoulesStateSpace::StateType>()->values, &qcur_[0], numDimensions_ * sizeof(double));
}
