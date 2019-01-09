/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Rice University
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

/* Author: Mark Moll */

#include <stdexcept>
#include <sstream>
#include "ompl/util/String.h"

namespace
{
    template <class RealType>
    RealType toReal(const std::string &s)
    {
        // convert from string using classic "C" locale semantics
        std::istringstream stream(s);
        stream.imbue(std::locale::classic());
        RealType result;
        stream >> result;
        if (stream.fail() || !stream.eof())
            throw std::runtime_error("Failed converting string to real number");
        return result;
    }

    template <class RealType>
    std::string toString(RealType t)
    {
        // convert to string using classic "C" locale semantics
        std::ostringstream stream;
        stream.imbue(std::locale::classic());
        stream << t;
        return stream.str();
    }
}  // namespace

float ompl::stof(const std::string &str)
{
    return toReal<float>(str);
}

double ompl::stod(const std::string &str)
{
    return toReal<double>(str);
}

long double ompl::stold(const std::string &str)
{
    return toReal<long double>(str);
}

std::string ompl::toString(float val)
{
    return ::toString(val);
}
std::string ompl::toString(double val)
{
    return ::toString(val);
}
std::string ompl::toString(long double val)
{
    return ::toString(val);
}
