/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rice University
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
*   * Neither the name of Rice University nor the names of its
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

/* Author: Ryan Luna */

#include "BoundedPlanarManipulatorStateSpace.h"
#include <ompl/util/Exception.h>

#include <boost/math/constants/constants.hpp>

#define PI boost::math::constants::pi<double>()
#define TWOPI boost::math::constants::two_pi<double>()

BoundedPlanarManipulatorStateSpace::BoundedPlanarManipulatorStateSpace(const PlanarManipulator* manip) : ompl::base::RealVectorStateSpace(manip->getNumLinks()), manip_(manip)
{
    cartDistance_ = false;
    cartInterpolator_ = false;
    hybridCartInterpolator_ = false;
}

BoundedPlanarManipulatorStateSpace::~BoundedPlanarManipulatorStateSpace()
{
}

bool BoundedPlanarManipulatorStateSpace::isMetricSpace() const
{
    return true; // Cartesian joint distance is metric???  IDK about triangle inequality
}

bool BoundedPlanarManipulatorStateSpace::hasSymmetricDistance() const
{
    return true;  // Cartesian joint distance is symmetric
}

bool BoundedPlanarManipulatorStateSpace::hasSymmetricInterpolate() const
{
    return !cartInterpolator_;  // cartesian interpolator is the least symmetric thing ever
}

double BoundedPlanarManipulatorStateSpace::distance(const ompl::base::State* s1, const ompl::base::State* s2) const
{
    if (cartDistance_)
    {
        // cartesian joint distance (approximation)
        const double* v1 = s1->as<StateType>()->values;
        const double* v2 = s2->as<StateType>()->values;

        std::vector<Eigen::Affine2d> f1, f2;
        manip_->FK(v1, f1);
        manip_->FK(v2, f2);

        double dist = 0.0;
        for(size_t i = 0; i < f1.size(); ++i)
            dist += (f1[i].translation() - f2[i].translation()).norm();
        return dist;
    }
    else
    {
        return ompl::base::RealVectorStateSpace::distance(s1, s2);
    }
}

void BoundedPlanarManipulatorStateSpace::interpolate(const ompl::base::State* from, const ompl::base::State* to, const double t, ompl::base::State* state) const
{
    const double* f = from->as<StateType>()->values;
    const double* too = to->as<StateType>()->values;
          double* out = state->as<StateType>()->values;

    if (cartInterpolator_)
        manip_->cartesianInterpolate(f, too, t, out);
    else if (hybridCartInterpolator_)
        manip_->hybridCartesianInterpolate(f, too, t, out);
    else
        ompl::base::RealVectorStateSpace::interpolate(from, to, t, state);
}

bool BoundedPlanarManipulatorStateSpace::usingCartesianInterpolator() const
{
    return cartInterpolator_;
}

void BoundedPlanarManipulatorStateSpace::useCartesianInterpolator(bool useIt)
{
    cartInterpolator_ = useIt;
}

bool BoundedPlanarManipulatorStateSpace::usingCartesianJointDistanceFn() const
{
    return cartDistance_;
}

void BoundedPlanarManipulatorStateSpace::useCartesianJointDistance(bool useIt)
{
    cartDistance_ = useIt;
}

bool BoundedPlanarManipulatorStateSpace::usingHybridCartesianInterpolator() const
{
    return hybridCartInterpolator_;
}

void BoundedPlanarManipulatorStateSpace::useHybridCartesianInterpolator(bool useIt)
{
    hybridCartInterpolator_ = useIt;
}