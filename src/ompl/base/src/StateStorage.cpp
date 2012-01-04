/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage
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

#include "ompl/base/StateStorage.h"

ompl::base::StateStorage::StateStorage(const StateSpacePtr &space) : space_(space)
{
}

ompl::base::StateStorage::~StateStorage(void)
{
    clear();
}

void ompl::base::StateStorage::load(const char *filename)
{
    std::ifstream in(filename, std::ios::binary);
    load(in);
    in.close();
}

void ompl::base::StateStorage::load(std::istream &in)
{
    clear();
    if (!in.good() || in.eof())
    {
	msg_.warn("Unable to load states");
	return;
    }
    
    // get length of file
    in.seekg(0, std::ios::end);
    std::size_t length = in.tellg();
    in.seekg(0, std::ios::beg);
    
    // load the file
    char *buffer = (char*)malloc(length);
    in.read(buffer, length);
    
    unsigned int l = space_->getSerializationLength();
    std::size_t nstates = length / l;
    msg_.debug("Deserializing %u states", nstates);
    states_.reserve(nstates);
    
    for (std::size_t i = 0 ; i < nstates ; ++i)
    {
	State *s = space_->allocState();
	space_->deserialize(s, buffer + i * l);
	addState(s);
    }
    free(buffer);
}

void ompl::base::StateStorage::store(const char *filename)
{
    std::ofstream out(filename, std::ios::binary);
    store(out);
    out.close();
}

void ompl::base::StateStorage::store(std::ostream &out)
{
    if (!out.good())
    {
	msg_.warn("Unable to store states");
	return;
    }

    msg_.debug("Serializing %u states", (unsigned int)states_.size());

    unsigned int l = space_->getSerializationLength();
    std::size_t length = l * states_.size();
    char *buffer = (char*)malloc(length);
    for (std::size_t i = 0 ; i < states_.size() ; ++i)
	space_->serialize(buffer + i * l, states_[i]);
    out.write(buffer, length);
    free(buffer);
}

void ompl::base::StateStorage::addState(const State *state)
{
    states_.push_back(state);
}

void ompl::base::StateStorage::clear(void)
{
    for (std::size_t i = 0 ; i < states_.size() ; ++i)
	space_->freeState(const_cast<State*>(states_[i]));
    states_.clear();
}
