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
#include "ompl/base/PrecomputedStateSampler.h"
#include "ompl/util/Exception.h"
#include <fstream>
#include <algorithm>

#include <boost/serialization/binary_object.hpp>
#include <boost/archive/archive_exception.hpp>
#include <utility>

/// @cond IGNORE
static ompl::base::StateSamplerPtr allocPrecomputedStateSampler(const ompl::base::StateSpace *space,
                                                                const std::vector<int> &expectedSignature,
                                                                const std::vector<const ompl::base::State *> *states,
                                                                std::size_t minIndex, std::size_t maxIndex)
{
    std::vector<int> sig;
    space->computeSignature(sig);
    if (sig != expectedSignature)
    {
        std::stringstream ss;
        ss << "Cannot allocate state sampler for a state space whose signature does not match that of the stored "
              "states. ";
        ss << "Expected signature ";
        for (int i : expectedSignature)
            ss << i << " ";
        ss << "but space " << space->getName() << " has signature ";
        for (int i : sig)
            ss << i << " ";
        throw ompl::Exception(ss.str());
    }
    return std::make_shared<ompl::base::PrecomputedStateSampler>(space, *states, minIndex, maxIndex);
}

static const std::uint_fast32_t OMPL_ARCHIVE_MARKER = 0x4C504D4F;  // this spells OMPL
/// @endcond

ompl::base::StateStorage::StateStorage(StateSpacePtr space) : space_(std::move(space)), hasMetadata_(false)
{
}

ompl::base::StateStorage::~StateStorage()
{
    freeMemory();
}

void ompl::base::StateStorage::load(const char *filename)
{
    std::ifstream in(filename, std::ios::binary);
    load(in);
    in.close();
}

void ompl::base::StateStorage::store(const char *filename)
{
    std::ofstream out(filename, std::ios::binary);
    store(out);
    out.close();
}

void ompl::base::StateStorage::load(std::istream &in)
{
    clear();
    if (!in.good() || in.eof())
    {
        OMPL_WARN("Unable to load states");
        return;
    }
    try
    {
        boost::archive::binary_iarchive ia(in);
        Header h;
        ia >> h;
        if (h.marker != OMPL_ARCHIVE_MARKER)
        {
            OMPL_ERROR("OMPL archive marker not found");
            return;
        }

        std::vector<int> sig;
        space_->computeSignature(sig);
        if (h.signature != sig)
        {
            OMPL_ERROR("State space signatures do not match");
            return;
        }
        loadStates(h, ia);
        loadMetadata(h, ia);
    }

    catch (boost::archive::archive_exception &ae)
    {
        OMPL_ERROR("Unable to load archive: %s", ae.what());
    }
}

void ompl::base::StateStorage::loadStates(const Header &h, boost::archive::binary_iarchive &ia)
{
    OMPL_DEBUG("Deserializing %u states", h.state_count);
    // load the file
    unsigned int l = space_->getSerializationLength();
    auto *buffer = new char[l];
    State *s = space_->allocState();
    for (std::size_t i = 0; i < h.state_count; ++i)
    {
        ia >> boost::serialization::make_binary_object(buffer, l);
        space_->deserialize(s, buffer);
        addState(s);
    }
    space_->freeState(s);
    delete[] buffer;
}

void ompl::base::StateStorage::loadMetadata(const Header & /*h*/, boost::archive::binary_iarchive & /*ia*/)
{
}

void ompl::base::StateStorage::store(std::ostream &out)
{
    if (!out.good())
    {
        OMPL_WARN("Unable to store states");
        return;
    }
    try
    {
        Header h;
        h.marker = OMPL_ARCHIVE_MARKER;
        h.state_count = states_.size();
        space_->computeSignature(h.signature);

        boost::archive::binary_oarchive oa(out);
        oa << h;

        storeStates(h, oa);
        storeMetadata(h, oa);
    }
    catch (boost::archive::archive_exception &ae)
    {
        OMPL_ERROR("Unable to save archive: %s", ae.what());
    }
}

void ompl::base::StateStorage::storeStates(const Header & /*h*/, boost::archive::binary_oarchive &oa)
{
    OMPL_DEBUG("Serializing %u states", (unsigned int)states_.size());

    unsigned int l = space_->getSerializationLength();
    auto *buffer = new char[l];
    for (auto &state : states_)
    {
        space_->serialize(buffer, state);
        oa << boost::serialization::make_binary_object(buffer, l);
    }
    delete[] buffer;
}

void ompl::base::StateStorage::storeMetadata(const Header & /*h*/, boost::archive::binary_oarchive & /*oa*/)
{
}

void ompl::base::StateStorage::addState(const State *state)
{
    State *copy = space_->allocState();
    space_->copyState(copy, state);
    states_.push_back(copy);
}

void ompl::base::StateStorage::generateSamples(unsigned int count)
{
    StateSamplerPtr ss = space_->allocStateSampler();
    states_.reserve(states_.size() + count);
    State *s = space_->allocState();
    for (unsigned int i = 0; i < count; ++i)
    {
        ss->sampleUniform(s);
        addState(s);
    }
    space_->freeState(s);
}

void ompl::base::StateStorage::freeMemory()
{
    for (auto &state : states_)
        space_->freeState(const_cast<State *>(state));
}

void ompl::base::StateStorage::clear()
{
    freeMemory();
    states_.clear();
}

void ompl::base::StateStorage::sort(const std::function<bool(const State *, const State *)> &op)
{
    std::sort(states_.begin(), states_.end(), op);
}

ompl::base::StateSamplerAllocator ompl::base::StateStorage::getStateSamplerAllocator() const
{
    return getStateSamplerAllocatorRange(0, states_.empty() ? 0 : states_.size() - 1);
}

ompl::base::StateSamplerAllocator ompl::base::StateStorage::getStateSamplerAllocatorRangeUntil(std::size_t until) const
{
    return getStateSamplerAllocatorRange(0, until);
}

ompl::base::StateSamplerAllocator ompl::base::StateStorage::getStateSamplerAllocatorRangeAfter(std::size_t after) const
{
    return getStateSamplerAllocatorRange(after, states_.empty() ? 0 : states_.size() - 1);
}

ompl::base::StateSamplerAllocator ompl::base::StateStorage::getStateSamplerAllocatorRange(std::size_t from,
                                                                                          std::size_t to) const
{
    if (states_.empty())
        throw Exception("Cannot allocate state sampler from empty state storage");
    std::vector<int> sig;
    space_->computeSignature(sig);
    return [this, sig, from, to](const ompl::base::StateSpace *space)
    {
        return allocPrecomputedStateSampler(space, sig, &states_, from, to);
    };
}

void ompl::base::StateStorage::print(std::ostream &out) const
{
    for (auto state : states_)
        space_->printState(state, out);
}
