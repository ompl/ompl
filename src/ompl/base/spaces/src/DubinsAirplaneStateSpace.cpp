/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Rice University
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

/* Original Author: Mark Moll, Airplane functionality added by Bendik Eger*/

#include "ompl/base/spaces/DubinsAirplaneStateSpace.h"
#include <boost/math/constants/constants.hpp>
#include <queue>
#include "ompl/base/SpaceInformation.h"
#include "ompl/util/Exception.h"

using namespace ompl::base;

namespace
{
    const double twopi = 2. * boost::math::constants::pi<double>();
    const double DUBINS_EPS = 1e-6;
    const double DUBINS_ZERO = -1e-7;

    inline double mod2pi(double x)
    {
        if (x < 0 && x > DUBINS_ZERO)
            return 0;
        double xm = x - twopi * floor(x / twopi);
        if (twopi - xm < .5 * DUBINS_EPS)
            xm = 0.;
        return xm;
    }

    DubinsAirplaneStateSpace::DubinsAirplanePath dubinsLSL(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        double tmp = 2. + d * d - 2. * (ca * cb + sa * sb - d * (sa - sb));
        if (tmp >= DUBINS_ZERO)
        {
            double theta = atan2(cb - ca, d + sa - sb);
            double t = mod2pi(-alpha + theta);
            double p = sqrt(std::max(tmp, 0.));
            double q = mod2pi(beta - theta);
            // assert(fabs(p * cos(alpha + t) - sa + sb - d) < 2 * DUBINS_EPS);
            // assert(fabs(p * sin(alpha + t) + ca - cb) < 2 * DUBINS_EPS);
            // assert(mod2pi(alpha + t + q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
            return DubinsAirplaneStateSpace::DubinsAirplanePath(DubinsAirplaneStateSpace::dubinsPathType[0], t, p, q);
        }
        return {};
    }

    DubinsAirplaneStateSpace::DubinsAirplanePath dubinsRSR(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        double tmp = 2. + d * d - 2. * (ca * cb + sa * sb - d * (sb - sa));
        if (tmp >= DUBINS_ZERO)
        {
            double theta = atan2(ca - cb, d - sa + sb);
            double t = mod2pi(alpha - theta);
            double p = sqrt(std::max(tmp, 0.));
            double q = mod2pi(-beta + theta);
            // assert(fabs(p * cos(alpha - t) + sa - sb - d) < 2 * DUBINS_EPS);
            // assert(fabs(p * sin(alpha - t) - ca + cb) < 2 * DUBINS_EPS);
            // assert(mod2pi(alpha - t - q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
            return DubinsAirplaneStateSpace::DubinsAirplanePath(DubinsAirplaneStateSpace::dubinsPathType[1], t, p, q);
        }
        return {};
    }

    DubinsAirplaneStateSpace::DubinsAirplanePath dubinsRSL(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        double tmp = d * d - 2. + 2. * (ca * cb + sa * sb - d * (sa + sb));
        if (tmp >= DUBINS_ZERO)
        {
            double p = sqrt(std::max(tmp, 0.));
            double theta = atan2(ca + cb, d - sa - sb) - atan2(2., p);
            double t = mod2pi(alpha - theta);
            double q = mod2pi(beta - theta);
            // assert(fabs(p * cos(alpha - t) - 2. * sin(alpha - t) + sa + sb - d) < 2 * DUBINS_EPS);
            // assert(fabs(p * sin(alpha - t) + 2. * cos(alpha - t) - ca - cb) < 2 * DUBINS_EPS);
            // assert(mod2pi(alpha - t + q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
            return DubinsAirplaneStateSpace::DubinsAirplanePath(DubinsAirplaneStateSpace::dubinsPathType[2], t, p, q);
        }
        return {};
    }

    DubinsAirplaneStateSpace::DubinsAirplanePath dubinsLSR(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        double tmp = -2. + d * d + 2. * (ca * cb + sa * sb + d * (sa + sb));
        if (tmp >= DUBINS_ZERO)
        {
            double p = sqrt(std::max(tmp, 0.));
            double theta = atan2(-ca - cb, d + sa + sb) - atan2(-2., p);
            double t = mod2pi(-alpha + theta);
            double q = mod2pi(-beta + theta);
            // assert(fabs(p * cos(alpha + t) + 2. * sin(alpha + t) - sa - sb - d) < 2 * DUBINS_EPS);
            // assert(fabs(p * sin(alpha + t) - 2. * cos(alpha + t) + ca + cb) < 2 * DUBINS_EPS);
            // assert(mod2pi(alpha + t - q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
            return DubinsAirplaneStateSpace::DubinsAirplanePath(DubinsAirplaneStateSpace::dubinsPathType[3], t, p, q);
        }
        return {};
    }

    DubinsAirplaneStateSpace::DubinsAirplanePath dubinsRLR(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        double tmp = .125 * (6. - d * d + 2. * (ca * cb + sa * sb + d * (sa - sb)));
        if (fabs(tmp) < 1.)
        {
            double p = twopi - acos(tmp);
            double theta = atan2(ca - cb, d - sa + sb);
            double t = mod2pi(alpha - theta + .5 * p);
            double q = mod2pi(alpha - beta - t + p);
            // assert(fabs(2. * sin(alpha - t + p) - 2. * sin(alpha - t) - d + sa - sb) < 2 * DUBINS_EPS);
            // assert(fabs(-2. * cos(alpha - t + p) + 2. * cos(alpha - t) - ca + cb) < 2 * DUBINS_EPS);
            // assert(mod2pi(alpha - t + p - q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
            return DubinsAirplaneStateSpace::DubinsAirplanePath(DubinsAirplaneStateSpace::dubinsPathType[4], t, p, q);
        }
        return {};
    }

    DubinsAirplaneStateSpace::DubinsAirplanePath dubinsLRL(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        double tmp = .125 * (6. - d * d + 2. * (ca * cb + sa * sb - d * (sa - sb)));
        if (fabs(tmp) < 1.)
        {
            double p = twopi - acos(tmp);
            double theta = atan2(-ca + cb, d + sa - sb);
            double t = mod2pi(-alpha + theta + .5 * p);
            double q = mod2pi(beta - alpha - t + p);
            // assert(fabs(-2. * sin(alpha + t - p) + 2. * sin(alpha + t) - d - sa + sb) < 2 * DUBINS_EPS);
            // assert(fabs(2. * cos(alpha + t - p) - 2. * cos(alpha + t) + ca - cb) < 2 * DUBINS_EPS);
            // assert(mod2pi(alpha + t - p + q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
            return DubinsAirplaneStateSpace::DubinsAirplanePath(DubinsAirplaneStateSpace::dubinsPathType[5], t, p, q);
        }
        return {};
    }

    DubinsAirplaneStateSpace::DubinsAirplanePath dubins(double d, double alpha, double beta)
    {
        if (d < DUBINS_EPS && fabs(alpha - beta) < DUBINS_EPS)
            return {DubinsAirplaneStateSpace::dubinsPathType[0], 0, d, 0};

        DubinsAirplaneStateSpace::DubinsAirplanePath path(dubinsLSL(d, alpha, beta)), tmp(dubinsRSR(d, alpha, beta));
        double len, minLength = path.length();

        if ((len = tmp.length()) < minLength)
        {
            minLength = len;
            path = tmp;
        }
        tmp = dubinsRSL(d, alpha, beta);
        if ((len = tmp.length()) < minLength)
        {
            minLength = len;
            path = tmp;
        }
        tmp = dubinsLSR(d, alpha, beta);
        if ((len = tmp.length()) < minLength)
        {
            minLength = len;
            path = tmp;
        }
        tmp = dubinsRLR(d, alpha, beta);
        if ((len = tmp.length()) < minLength)
        {
            minLength = len;
            path = tmp;
        }
        tmp = dubinsLRL(d, alpha, beta);
        if ((len = tmp.length()) < minLength)
            path = tmp;

        return path;
    }
}  // namespace

const ompl::base::DubinsAirplaneStateSpace::DubinsAirplanePathSegmentType
    ompl::base::DubinsAirplaneStateSpace::dubinsPathType[6][3] = {
        {DUBINS_LEFT, DUBINS_STRAIGHT, DUBINS_LEFT},  {DUBINS_RIGHT, DUBINS_STRAIGHT, DUBINS_RIGHT},
        {DUBINS_RIGHT, DUBINS_STRAIGHT, DUBINS_LEFT}, {DUBINS_LEFT, DUBINS_STRAIGHT, DUBINS_RIGHT},
        {DUBINS_RIGHT, DUBINS_LEFT, DUBINS_RIGHT},    {DUBINS_LEFT, DUBINS_RIGHT, DUBINS_LEFT}};

double ompl::base::DubinsAirplaneStateSpace::distance(const State *state1, const State *state2) const
{
    DubinsAirplaneStateSpace::DubinsAirplanePath path;

    path = dubins(state1, state2);
    double l = rho_ * path.length() / cos(path.climbAngle_);

    if (path.helix_)
        l = (rho_ * path.length() + path.helix_->length()) / cos(path.climbAngle_);

    if (isSymmetric_)
    {
        auto path2 = dubins(state2, state1);
        double l2 = rho_ * path2.length() / cos(path2.climbAngle_);

        if (path2.helix_)
            l2 = (rho_ * path2.length() + path2.helix_->length()) / cos(path2.climbAngle_);

        if (l2 < l)
            l = l2;
    }

    return l;
}

void ompl::base::DubinsAirplaneStateSpace::interpolate(const State *from, const State *to, const double t,
                                                       State *state) const
{
    bool firstTime = true;
    DubinsAirplanePath path;
    interpolate(from, to, t, firstTime, path, state);
}

void ompl::base::DubinsAirplaneStateSpace::interpolate(const State *from, const State *to, const double t,
                                                       bool &firstTime, DubinsAirplanePath &path, State *state) const
{
    if (firstTime)
    {
        if (t >= 1.)
        {
            if (to != state)
                copyState(state, to);
            return;
        }
        if (t <= 0.)
        {
            if (from != state)
                copyState(state, from);
            return;
        }

        path = dubins(from, to);
        if (isSymmetric_)
        {
            DubinsAirplanePath path2(dubins(to, from));
            if (path2.length() < path.length())
            {
                path2.reverse_ = true;
                path = path2;
            }
        }
        firstTime = false;
    }
    interpolate(from, path, t, state);
}

void ompl::base::DubinsAirplaneStateSpace::interpolate(const State *from, const DubinsAirplanePath &path, double t,
                                                       State *state) const
{
    SimpleSE3StateSpace::StateType *s = allocState()->as<StateType>();
    double seg = t * (path.length() * rho_), phi, v;

    if (path.helix_)
    {
        seg = t * (path.length() * rho_ + path.helix_->projectedLength());
    }

    s->setXY(0., 0.);
    s->setZ(seg * tan(path.climbAngle_));
    s->setYaw(from->as<StateType>()->getYaw());
    if (!path.reverse_)
    {
        for (unsigned int i = 0; i < 3 && seg > 0; ++i)
        {
            v = std::min(seg, path.length_[i] * rho_);
            phi = s->getYaw();
            seg -= v;
            v = v / rho_;
            switch (path.type_[i])
            {
                case DUBINS_LEFT:
                    s->setXY(s->getX() + (sin(phi + v) - sin(phi)) * rho_,
                             s->getY() - (cos(phi + v) - cos(phi)) * rho_);
                    s->setYaw(phi + v);
                    break;
                case DUBINS_RIGHT:
                    s->setXY(s->getX() - (sin(phi - v) - sin(phi)) * rho_,
                             s->getY() + (cos(phi - v) - cos(phi)) * rho_);
                    s->setYaw(phi - v);
                    break;
                case DUBINS_STRAIGHT:
                    s->setXY(s->getX() + (v * cos(phi)) * rho_, s->getY() + (v * sin(phi)) * rho_);
                    break;
            }
        }
        if (path.helix_)
        {
            v = seg / path.helix_->rho_;
            phi = s->getYaw();
            switch (path.helix_->type)
            {
                case DUBINS_LEFT:
                    s->setXY(s->getX() + (sin(phi + v) - sin(phi)) * path.helix_->rho_,
                             s->getY() - (cos(phi + v) - cos(phi)) * path.helix_->rho_);
                    s->setYaw(phi + v);
                    break;
                case DUBINS_RIGHT:
                    s->setXY(s->getX() - (sin(phi - v) - sin(phi)) * path.helix_->rho_,
                             s->getY() + (cos(phi - v) - cos(phi)) * path.helix_->rho_);
                    s->setYaw(phi - v);
                    break;
                default:
                    break;
            }
        }
    }
    else
    {
        if (path.helix_)
        {
            v = seg / path.helix_->rho_;
            phi = s->getYaw();
            switch (path.helix_->type)
            {
                case DUBINS_LEFT:
                    s->setXY(s->getX() + (sin(phi - v) - sin(phi)) * path.helix_->rho_,
                             s->getY() - (cos(phi - v) - cos(phi)) * path.helix_->rho_);
                    s->setYaw(phi - v);
                    break;
                case DUBINS_RIGHT:
                    s->setXY(s->getX() - (sin(phi + v) - sin(phi)) * path.helix_->rho_,
                             s->getY() + (cos(phi + v) - cos(phi)) * path.helix_->rho_);
                    s->setYaw(phi + v);
                    break;
                default:
                    break;
            }
        }
        for (unsigned int i = 0; i < 3 && seg > 0; ++i)
        {
            v = std::min(seg, path.length_[2 - i]);
            phi = s->getYaw();
            seg -= v;
            switch (path.type_[2 - i])
            {
                case DUBINS_LEFT:
                    s->setXY(s->getX() + (sin(phi - v) - sin(phi)) * rho_,
                             s->getY() - (cos(phi - v) - cos(phi)) * rho_);
                    s->setYaw(phi - v);
                    break;
                case DUBINS_RIGHT:
                    s->setXY(s->getX() - (sin(phi + v) - sin(phi)) * rho_,
                             s->getY() + (cos(phi + v) - cos(phi)) * rho_);
                    s->setYaw(phi + v);
                    break;
                case DUBINS_STRAIGHT:
                    s->setXY(s->getX() - (v * cos(phi)) * rho_, s->getY() - (v * sin(phi)) * rho_);
                    break;
            }
        }
    }

    state->as<StateType>()->setX(s->getX() + from->as<StateType>()->getX());
    state->as<StateType>()->setY(s->getY() + from->as<StateType>()->getY());
    state->as<StateType>()->setZ(s->getZ() + from->as<StateType>()->getZ());
    getSubspace(1)->enforceBounds(s->as<SO2StateSpace::StateType>(1));
    state->as<StateType>()->setYaw(s->getYaw());
    freeState(s);
}

ompl::base::DubinsAirplaneStateSpace::DubinsAirplanePath
ompl::base::DubinsAirplaneStateSpace::dubins(const State *state1, const State *state2) const
{
    const auto *s1 = static_cast<const StateType *>(state1);
    const auto *s2 = static_cast<const StateType *>(state2);
    double x1 = s1->getX(), y1 = s1->getY(), z1 = s1->getZ(), th1 = s1->getYaw();
    double x2 = s2->getX(), y2 = s2->getY(), z2 = s2->getZ(), th2 = s2->getYaw();
    double dx = x2 - x1, dy = y2 - y1, dz = z2 - z1, d = sqrt(dx * dx + dy * dy) / rho_, th = atan2(dy, dx);
    double alpha = mod2pi(th1 - th), beta = mod2pi(th2 - th);
    DubinsAirplaneStateSpace::DubinsAirplanePath path = ::dubins(d, alpha, beta);

    double ca = atan2(dz, path.length() * rho_);
    if (fabs(ca) < climbAngleLimit_)  // LOW PATH
    {
        path.climbAngle_ = ca;
    }
    else
    {
        double n = floor((fabs(dz) / tan(climbAngleLimit_) - path.length() * rho_) / (2 * M_PI * rho_));
        if (n)  // HIGH PATH
        {
            double r =
                (fabs(dz) - path.length() * rho_ * tan(climbAngleLimit_)) / (2 * M_PI * n * tan(climbAngleLimit_));
            double ca = copysign(climbAngleLimit_, dz);
            path.climbAngle_ = ca;
            path.helix_ = new Helix(path.type_[2], r, ca, n);
        }
        else  // INTERMEDIATE PATH
        {
            n = 1;
            double ca = atan2(dz, path.length() * rho_ + rho_ * n * 2 * M_PI);
            path.climbAngle_ = ca;
            path.helix_ = new Helix(path.type_[2], rho_, ca, n);
        }
    }
    return path;
}

void ompl::base::DubinsAirplaneMotionValidator::defaultSettings()
{
    stateSpace_ = dynamic_cast<DubinsAirplaneStateSpace *>(si_->getStateSpace().get());
    if (stateSpace_ == nullptr)
        throw Exception("No state space for motion validator");
}

bool ompl::base::DubinsAirplaneMotionValidator::checkMotion(const State *s1, const State *s2,
                                                            std::pair<State *, double> &lastValid) const
{
    /* assume motion starts in a valid configuration so s1 is valid */

    bool result = true, firstTime = true;
    DubinsAirplaneStateSpace::DubinsAirplanePath path;
    int nd = stateSpace_->validSegmentCount(s1, s2);

    if (nd > 1)
    {
        /* temporary storage for the checked state */
        State *test = si_->allocState();

        for (int j = 1; j < nd; ++j)
        {
            stateSpace_->interpolate(s1, s2, (double)j / (double)nd, firstTime, path, test);
            if (!si_->isValid(test))
            {
                lastValid.second = (double)(j - 1) / (double)nd;
                if (lastValid.first != nullptr)
                    stateSpace_->interpolate(s1, s2, lastValid.second, firstTime, path, lastValid.first);
                result = false;
                break;
            }
        }
        si_->freeState(test);
    }

    if (result)
        if (!si_->isValid(s2))
        {
            lastValid.second = (double)(nd - 1) / (double)nd;
            if (lastValid.first != nullptr)
                stateSpace_->interpolate(s1, s2, lastValid.second, firstTime, path, lastValid.first);
            result = false;
        }

    if (result)
        valid_++;
    else
        invalid_++;

    return result;
}

bool ompl::base::DubinsAirplaneMotionValidator::checkMotion(const State *s1, const State *s2) const
{
    /* assume motion starts in a valid configuration so s1 is valid */
    if (!si_->isValid(s2))
        return false;

    bool result = true, firstTime = true;
    DubinsAirplaneStateSpace::DubinsAirplanePath path;
    int nd = stateSpace_->validSegmentCount(s1, s2);

    /* initialize the queue of test positions */
    std::queue<std::pair<int, int>> pos;
    if (nd >= 2)
    {
        pos.emplace(1, nd - 1);

        /* temporary storage for the checked state */
        State *test = si_->allocState();

        /* repeatedly subdivide the path segment in the middle (and check the middle) */
        while (!pos.empty())
        {
            std::pair<int, int> x = pos.front();

            int mid = (x.first + x.second) / 2;
            stateSpace_->interpolate(s1, s2, (double)mid / (double)nd, firstTime, path, test);

            if (!si_->isValid(test))
            {
                result = false;
                break;
            }

            pos.pop();

            if (x.first < mid)
                pos.emplace(x.first, mid - 1);
            if (x.second > mid)
                pos.emplace(mid + 1, x.second);
        }

        si_->freeState(test);
    }

    if (result)
        valid_++;
    else
        invalid_++;

    return result;
}
