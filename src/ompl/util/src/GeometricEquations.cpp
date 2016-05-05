/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, University of Toronto
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
*   * Neither the name of the University of Toronto nor the names of its
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

/* Author: Jonathan Gammell*/

// This file's header
#include "ompl/util/GeometricEquations.h"

// For gamma function
#include <cmath>

// For pi definition
#include <boost/math/constants/constants.hpp>

// OMPL exceptions
#include "ompl/util/Exception.h"

double ompl::nBallMeasure(unsigned int N, double r)
{
    return std::pow(std::sqrt(boost::math::constants::pi<double>()) * r, static_cast<double>(N)) /
           std::tgamma(static_cast<double>(N) / 2.0 + 1.0);
}

double ompl::unitNBallMeasure(unsigned int N)
{
    // This is the radius version with r removed (as it is 1) for efficiency
    return std::pow(std::sqrt(boost::math::constants::pi<double>()), static_cast<double>(N)) /
           std::tgamma(static_cast<double>(N) / 2.0 + 1.0);
}

double ompl::prolateHyperspheroidMeasure(unsigned int N, double dFoci, double dTransverse)
{
    // Sanity check input
    if (dTransverse < dFoci)
    {
        throw Exception("Transverse diameter cannot be less than the minimum transverse diameter.");
    }

    // Variable
    // The conjugate diameter:
    double conjugateDiameter;
    // The Lebesgue measure return value
    double lmeas;

    // Calculate the conjugate diameter:
    conjugateDiameter = std::sqrt(dTransverse * dTransverse - dFoci * dFoci);

    // Calculate the volume
    // First multiply together the radii, noting that the first radius is the transverse diameter/2.0, and the other N-1
    // are the conjugate diameter/2.0
    lmeas = dTransverse / 2.0;
    for (unsigned int i = 1u; i < N; ++i)
    {
        lmeas = lmeas * conjugateDiameter / 2.0;
    }

    // Then multiply by the volume of the unit n-ball.
    lmeas = lmeas * unitNBallMeasure(N);

    // Finally return:
    return lmeas;
}
