/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Rice University
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

/* Author: Matt Maly */

#include "ompl/control/planners/ltl/World.h"
#include "ompl/util/Console.h"
#include "ompl/util/Hash.h"
#include <unordered_map>
#include <string>

ompl::control::World::World(unsigned int np) : numProps_(np)
{
}

bool ompl::control::World::operator[](unsigned int i) const
{
    std::unordered_map<unsigned int, bool>::const_iterator p = props_.find(i);
    if (p == props_.end())
        OMPL_ERROR("Proposition %u is not set in world", i);
    return p->second;
}

bool &ompl::control::World::operator[](unsigned int i)
{
    return props_[i];
}

unsigned int ompl::control::World::numProps() const
{
    return numProps_;
}

bool ompl::control::World::satisfies(const World &w) const
{
    std::unordered_map<unsigned int, bool>::const_iterator p, q;
    for (p = w.props_.begin(); p != w.props_.end(); ++p)
    {
        q = props_.find(p->first);
        if (q == props_.end() || *q != *p)
            return false;
    }
    return true;
}

std::string ompl::control::World::formula(void) const
{
    if (props_.empty())
        return "true";
    std::unordered_map<unsigned int, bool>::const_iterator p = props_.begin();
    std::string f = std::string(p->second ? "p" : "!p") + std::to_string(p->first);
    ++p;
    for (; p != props_.end(); ++p)
        f += std::string(p->second ? " & p" : " & !p") + std::to_string(p->first);
    return f;
}

const std::unordered_map<unsigned int, bool> &ompl::control::World::props(void) const
{
    return props_;
}

bool ompl::control::World::operator==(const World &w) const
{
    return numProps_ == w.numProps_ && props_ == w.props_;
}

void ompl::control::World::clear(void)
{
    props_.clear();
}

size_t std::hash<ompl::control::World>::operator()(const ompl::control::World &w) const
{
    std::size_t hash = 0;
    std::unordered_map<unsigned int, bool>::const_iterator p;
    for (p = w.props_.begin(); p != w.props_.end(); ++p)
        ompl::hash_combine(hash, *p);
    return hash;
}
