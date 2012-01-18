/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage
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
*   * Neither the name of the Willow Garage nor the names of its
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

#include "ompl/tools/spaces/StateAddress.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"

ompl::StateAddress::StateAddress(void)
{
}

ompl::StateAddress::StateAddress(const base::StateSpacePtr &space)
{
    setStateSpace(space);
}

ompl::StateAddress::~StateAddress(void)
{
}

void ompl::StateAddress::setStateSpace(const base::StateSpacePtr &space)
{
    space_ = space;
    Location loc;
    storeLocation(loc, space.get());
}

void ompl::StateAddress::storeLocation(Location loc, const base::StateSpace *s)
{
    base::State *test = s->allocState();
    if (s->getValueAddressAtIndex(test, 0) != NULL)
    {
        loc.index = 0;
        loc.space = s;
        locations_[s->getName()] = loc;
    }
    s->freeState(test);

    if (s->isCompound())
        for (unsigned int i = 0 ; i < s->as<base::CompoundStateSpace>()->getSubSpaceCount() ; ++i)
        {
            loc.chain.push_back(i);
            storeLocation(loc, s->as<base::CompoundStateSpace>()->getSubSpace(i).get());
            loc.chain.pop_back();
        }
    else
        if (s->getType() == base::STATE_SPACE_REAL_VECTOR)
            for (unsigned int i = 0 ; i < s->getDimension() ; ++i)
            {
                const std::string &name = s->as<base::RealVectorStateSpace>()->getDimensionName(i);
                if (!name.empty())
                {
                    loc.index = i;
                    locations_[name] = loc;
                }
            }
}

double* ompl::StateAddress::getValueAddressAtLocation(const Location &loc, base::State *state) const
{
    std::size_t index = 0;
    while (loc.chain.size() > index)
        state = state->as<base::CompoundState>()->components[loc.chain[index++]];
    return loc.space->getValueAddressAtIndex(state, loc.index);
}

const double* ompl::StateAddress::getValueAddressAtLocation(const Location &loc, const base::State *state) const
{
    return getValueAddressAtLocation(loc, const_cast<base::State*>(state));
}

double* ompl::StateAddress::getValueAddressAtName(const std::string &name, base::State *state) const
{
    std::map<std::string, Location>::const_iterator it = locations_.find(name);
    if (it == locations_.end())
    {
        msg_.error("Name '%s' is not known for space '%s'", name.c_str(), space_->getName().c_str());
        return NULL;
    }
    else
        return getValueAddressAtLocation(it->second, state);
}

const double* ompl::StateAddress::getValueAddressAtName(const std::string &name, const base::State *state) const
{
    return getValueAddressAtName(name, const_cast<base::State*>(state));
}
