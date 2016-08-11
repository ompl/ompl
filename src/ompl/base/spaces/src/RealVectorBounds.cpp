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

/* Author: Ioan Sucan */

#include "ompl/base/spaces/RealVectorBounds.h"
#include "ompl/util/Exception.h"
#include <algorithm>
#include <limits>

void ompl::base::RealVectorBounds::setLow(double value)
{
    std::fill(low.begin(), low.end(), value);
}

void ompl::base::RealVectorBounds::setHigh(double value)
{
    std::fill(high.begin(), high.end(), value);
}

void ompl::base::RealVectorBounds::setLow(unsigned int index, double value)
{
    low[index] = value;
}

void ompl::base::RealVectorBounds::setHigh(unsigned int index, double value)
{
    high[index] = value;
}

std::vector<double> ompl::base::RealVectorBounds::getDifference() const
{
    unsigned int n = std::min(low.size(), high.size());
    std::vector<double> result(n);
    for (unsigned int i = 0; i < n; ++i)
        result[i] = high[i] - low[i];
    return result;
}

double ompl::base::RealVectorBounds::getVolume() const
{
    double v = 1.0;
    unsigned int n = std::min(low.size(), high.size());
    for (unsigned int i = 0; i < n; ++i)
    {
        double d = high[i] - low[i];
        v *= d;
    }
    return v;
}

void ompl::base::RealVectorBounds::resize(std::size_t size)
{
    low.resize(size, 0.0);
    high.resize(size, 0.0);
}

void ompl::base::RealVectorBounds::check() const
{
    if (low.size() != high.size())
        throw Exception("Lower and upper bounds are not of same dimension");
    for (unsigned int i = 0; i < low.size(); ++i)
        if (low[i] > high[i])
            throw Exception("Bounds for real vector space seem to be incorrect (lower bound must be stricly less than "
                            "upper bound). Sampling will not be possible");
}
