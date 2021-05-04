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

#include <ompl/base/spaces/special/KleinBottleStateSpace.h>
#include <ompl/tools/config/MagicConstants.h>
#include <cstring>
#include <boost/math/constants/constants.hpp>

using namespace boost::math::double_constants;  // pi
using namespace ompl::base;

KleinBottleStateSampler::KleinBottleStateSampler(const StateSpace *space) : StateSampler(space)
{
}

void KleinBottleStateSampler::sampleUniform(State *state)
{
    bool acceptedSampleFound = false;
    while (!acceptedSampleFound)
    {
        double u = rng_.uniformReal(0, pi);
        double v = rng_.uniformReal(-pi, pi);

        // NOTE: The idea here is to compute the norm of the gradient at each
        // point of the surface (i.e. the gradient of the coordinate mapping from
        //(u,v) to (x,y,z)). To get the norm, we divide by the maximum norm of the
        // gradient over the whole surface. This gives a number between [0,1]. We
        // then do rejection sampling, by choosing a random number in [0,1] and
        // accept if the norm is larger than this random number. Surface elements
        // with a high curvature will have a small norm and will therefore be
        // penalized under this method (i.e. rejected more often).
        // See also: https://mathematica.stackexchange.com/questions/148693/generating-random-points-on-a-kleins-bottle

        // NOTE: Automatic differential via sympy script
        double cu = cos(u);
        double cv = cos(v);
        double su = sin(u);
        double sv = sin(v);
        double cu3 = powf(cu, 3);
        double cu5 = powf(cu, 5);
        double cu6 = powf(cu, 6);
        double cu7 = powf(cu, 7);
        double cu8 = powf(cu, 8);

        double su2 = powf(su, 2);
        double su3 = powf(su, 3);
        double su4 = powf(su, 4);
        double su5 = powf(su, 5);
        double su6 = powf(su, 6);
        double su7 = powf(su, 7);
        double su8 = powf(su, 8);

        double aprime =
            (64.0 * su8 - 128.0 * su6 + 60.0 * su4 + 0.4 * su * cv - (1 / 6) * cu * cv - 0.5 * cos(3 * u) * cv);

        double a = (-aprime * cv + (2 / 3) * sv * sv * cu * cos(2 * u));

        double bprime = ((26 + 2 / 3.0) * su7 * cv - 55.0 * su5 * cv - (37 + 1.0 / 3.0) * su3 * cu6 * cv +
                         28.0 * su3 * cv + (10 + 2 / 3.0) * su * cu8 * cv - (10 + 2 / 3.0) * su * cu6 * cv -
                         4.0 * sin(2 * u) + 22.4 * cu7 * cv - 35.2 * cu5 * cv + 12.2 * cu3 * cv + 0.6 * cu * cv);

        double cprime =
            ((5 + 1 / 3.0) * su5 * cu + 3.2 * su4 - (10 + 2 / 3.0) * su3 * cu - 6.4 * su2 + 2.5 * sin(2 * u) + 3.0);

        double b = (((1.0 / 3.0) * sin(2 * u) + 0.4) * bprime * cu - cprime * aprime * su3);

        double c = ((5.0 / 6.0) * sin(2 * u) + 1);

        double d =
            (-((1.0 / 3.0) * sin(2 * u) + 0.4) * bprime * cv + (2.0 / 3.0) * cprime * su3 * sv * sv * cos(2 * u));

        double s = sqrtf(a * a * (0.16 * c * c) + b * b * sv * sv + d * d);

        s = s / gMax_;

        double mu = rng_.uniformReal(0, 1);
        if (mu <= s)
        {
            KleinBottleStateSpace::StateType *K = state->as<KleinBottleStateSpace::StateType>();
            K->setUV(u, v);
            acceptedSampleFound = true;
        }
    }
}

void KleinBottleStateSampler::sampleUniformNear(State *state, const State *near, double distance)
{
    KleinBottleStateSpace::StateType *K = state->as<KleinBottleStateSpace::StateType>();
    const KleinBottleStateSpace::StateType *Knear = near->as<KleinBottleStateSpace::StateType>();
    K->setU(rng_.uniformReal(Knear->getU() - distance, Knear->getU() + distance));
    K->setV(rng_.uniformReal(Knear->getV() - distance, Knear->getV() + distance));
    space_->enforceBounds(state);
}

void KleinBottleStateSampler::sampleGaussian(State *state, const State *mean, double stdDev)
{
    KleinBottleStateSpace::StateType *K = state->as<KleinBottleStateSpace::StateType>();
    const KleinBottleStateSpace::StateType *Kmean = mean->as<KleinBottleStateSpace::StateType>();
    K->setU(rng_.gaussian(Kmean->getU(), stdDev));
    K->setV(rng_.gaussian(Kmean->getV(), stdDev));

    space_->enforceBounds(state);
}

KleinBottleStateSpace::KleinBottleStateSpace()
{
    setName("KleinBottle" + getName());
    type_ = STATE_SPACE_KLEIN_BOTTLE;

    // We model the Klein bottle as a regular cylinder, but where both ends are
    // glued inversely together. For more information, check out the
    // wikipedia article: https://en.wikipedia.org/wiki/Klein_bottle.
    // Both interpolation and distance computation have to take
    // the gluing into account when crossing over the boundary.
    // ------<-------
    // |            |
    // |            |
    // v            v u-dimension (0 to pi)
    // |            |
    // |            |
    // ------>-------
    //  v-dimension (0 to 2*pi)
    //
    //  Gluing:
    // u=pi+0.001:  0 ----------- -pi pi ---------- 0
    // u=0       :  -pi ----------- 0 0 ---------- pi

    StateSpacePtr R1(std::make_shared<RealVectorStateSpace>(1));
    R1->as<RealVectorStateSpace>()->setBounds(0, pi);

    StateSpacePtr SO2(std::make_shared<SO2StateSpace>());

    addSubspace(R1, 1.0);
    addSubspace(SO2, 1.0);

    lock();
}

StateSamplerPtr KleinBottleStateSpace::allocDefaultStateSampler() const
{
    return std::make_shared<KleinBottleStateSampler>(this);
}

double KleinBottleStateSpace::distance(const State *state1, const State *state2) const
{
    const double u1 = state1->as<KleinBottleStateSpace::StateType>()->getU();
    const double u2 = state2->as<KleinBottleStateSpace::StateType>()->getU();

    double diffU = u2 - u1;

    if (fabs(diffU) <= 0.5 * pi)
    {
        return CompoundStateSpace::distance(state1, state2);
    }
    else
    {
        double d_u = pi - fabs(diffU);

        const double v1 = state1->as<KleinBottleStateSpace::StateType>()->getV();
        double v2 = state2->as<KleinBottleStateSpace::StateType>()->getV();

        // reverse v2 (valid for both directions)
        v2 = (v2 > 0 ? pi - v2 : -pi - v2);

        double d_v = fabs(v2 - v1);
        d_v = (d_v > pi) ? 2.0 * pi - d_v : d_v;

        double dist = d_u + d_v;

        return dist;
    }
}

void KleinBottleStateSpace::interpolate(const State *from, const State *to, double t, State *state) const
{
    double u1 = from->as<KleinBottleStateSpace::StateType>()->getU();
    double u2 = to->as<KleinBottleStateSpace::StateType>()->getU();

    double diffU = u2 - u1;

    if (fabs(diffU) <= 0.5 * pi)
    {
        // interpolate as if it would be a cylinder
        CompoundStateSpace::interpolate(from, to, t, state);
    }
    else
    {
        // Interpolate along u-dimension
        if (diffU > 0.0)
        {
            diffU = pi - diffU;
        }
        else
        {
            diffU = -pi - diffU;
        }

        double u = u1 - diffU * t;

        bool crossed = false;
        if (u > pi)
        {
            u -= pi;
            crossed = true;
        }
        else if (u < 0)
        {
            u += pi;
            crossed = true;
        }

        state->as<KleinBottleStateSpace::StateType>()->setU(u);

        double v1 = from->as<KleinBottleStateSpace::StateType>()->getV();
        double v2 = to->as<KleinBottleStateSpace::StateType>()->getV();

        // If we crossed the gluing, we need to invert the "from"-state, otherwise
        // we need to invert the "to"-state (similar to default SO2 interpolation)
        if (crossed)
        {
            v1 = (v1 > 0 ? pi - v1 : -pi - v1);
        }
        else
        {
            v2 = (v2 > 0 ? pi - v2 : -pi - v2);
        }

        double diffV = v2 - v1;
        double v = 0;

        if (fabs(diffV) <= pi)
        {
            v = v1 + diffV * t;
        }
        else
        {
            if (diffV > 0.0)
                diffV = 2.0 * pi - diffV;
            else
                diffV = -2.0 * pi - diffV;
            v = v1 - diffV * t;

            if (v > pi)
                v -= 2.0 * pi;
            else if (v < -pi)
                v += 2.0 * pi;
        }

        state->as<KleinBottleStateSpace::StateType>()->setV(v);
    }
}

State *KleinBottleStateSpace::allocState() const
{
    auto *state = new StateType();
    allocStateComponents(state);
    return state;
}

Eigen::Vector3f KleinBottleStateSpace::toVector(const State *state) const
{
    // Formula from https://en.wikipedia.org/wiki/Klein_bottle#Bottle_shape
    const KleinBottleStateSpace::StateType *s = state->as<KleinBottleStateSpace::StateType>();
    float u = s->getU();
    float v = s->getV() + pi;  // NOTE: SO2 state space has bounds [-pi, +pi]

    assert(u >= 0);
    assert(u <= pi);
    assert(v >= 0);
    assert(v <= 2 * pi);

    double cu = cos(u);
    double cv = cos(v);
    double su = sin(u);
    double sv = sin(v);
    double cu2 = powf(cu, 2);
    double cu3 = powf(cu, 3);
    double cu4 = powf(cu, 4);
    double cu5 = powf(cu, 5);
    double cu6 = powf(cu, 6);
    double cu7 = powf(cu, 7);

    double a = 3 * cv - 30 * su + 90 * cu4 * su - 60 * cu6 * su + 5 * cu * cv * su;

    Eigen::Vector3f q;
    q[0] = -2.0 / 15.0 * cu * a;

    double b = 3 * cv - 3 * cu2 * cv - 48 * cu4 * cv + 48 * cu6 * cv - 60 * su + 5 * cu * cv * su - 5 * cu3 * cv * su -
               80 * cu5 * cv * su + 80 * cu7 * cv * su;

    q[1] = -1.0 / 15.0 * su * b;

    q[2] = 2.0 / 15.0 * (3 + 5 * cu * su) * sv;

    return q;
}
