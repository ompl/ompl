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

#include "ompl/base/StateSpace.h"
#include "ompl/util/Exception.h"
#include "ompl/tools/config/MagicConstants.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/util/String.h"
#include <mutex>
#include <boost/scoped_ptr.hpp>
#include <numeric>
#include <limits>
#include <queue>
#include <cmath>
#include <list>
#include <set>

const std::string ompl::base::StateSpace::DEFAULT_PROJECTION_NAME = "";

/// @cond IGNORE
namespace ompl
{
    namespace base
    {
        namespace
        {
            struct AllocatedSpaces
            {
                AllocatedSpaces() = default;
                std::list<StateSpace *> list_;
                std::mutex lock_;
                unsigned int counter_{0};
            };

            static boost::scoped_ptr<AllocatedSpaces> g_allocatedSpaces;
            static std::once_flag g_once;

            void initAllocatedSpaces()
            {
                g_allocatedSpaces.reset(new AllocatedSpaces);
            }

            AllocatedSpaces &getAllocatedSpaces()
            {
                std::call_once(g_once, &initAllocatedSpaces);
                return *g_allocatedSpaces;
            }
        }  // namespace
    }
}
/// @endcond

ompl::base::StateSpace::StateSpace()
{
    AllocatedSpaces &as = getAllocatedSpaces();
    std::lock_guard<std::mutex> smLock(as.lock_);

    // autocompute a unique name
    name_ = "Space" + std::to_string(as.counter_++);

    longestValidSegment_ = 0.0;
    longestValidSegmentFraction_ = 0.01;  // 1%
    longestValidSegmentCountFactor_ = 1;

    type_ = STATE_SPACE_UNKNOWN;

    maxExtent_ = std::numeric_limits<double>::infinity();

    params_.declareParam<double>("longest_valid_segment_fraction",
                                 [this](double segmentFraction) { setLongestValidSegmentFraction(segmentFraction); },
                                 [this] { return getLongestValidSegmentFraction(); });

    params_.declareParam<unsigned int>("valid_segment_count_factor",
                                       [this](unsigned int factor) { setValidSegmentCountFactor(factor); },
                                       [this] { return getValidSegmentCountFactor(); });
    as.list_.push_back(this);
}

ompl::base::StateSpace::~StateSpace()
{
    AllocatedSpaces &as = getAllocatedSpaces();
    std::lock_guard<std::mutex> smLock(as.lock_);
    as.list_.remove(this);
}

/// @cond IGNORE
namespace ompl
{
    namespace base
    {
        static void computeStateSpaceSignatureHelper(const StateSpace *space, std::vector<int> &signature)
        {
            signature.push_back(space->getType());
            signature.push_back(space->getDimension());

            if (space->isCompound())
            {
                unsigned int c = space->as<CompoundStateSpace>()->getSubspaceCount();
                for (unsigned int i = 0; i < c; ++i)
                    computeStateSpaceSignatureHelper(space->as<CompoundStateSpace>()->getSubspace(i).get(), signature);
            }
        }

        void computeLocationsHelper(const StateSpace *s,
                                    std::map<std::string, StateSpace::SubstateLocation> &substateMap,
                                    std::vector<StateSpace::ValueLocation> &locationsArray,
                                    std::map<std::string, StateSpace::ValueLocation> &locationsMap,
                                    StateSpace::ValueLocation loc)
        {
            loc.stateLocation.space = s;
            substateMap[s->getName()] = loc.stateLocation;
            State *test = s->allocState();
            if (s->getValueAddressAtIndex(test, 0) != nullptr)
            {
                loc.index = 0;
                locationsMap[s->getName()] = loc;
                // if the space is compound, we will find this value again in the first subspace
                if (!s->isCompound())
                {
                    if (s->getType() == base::STATE_SPACE_REAL_VECTOR)
                    {
                        const std::string &name = s->as<base::RealVectorStateSpace>()->getDimensionName(0);
                        if (!name.empty())
                            locationsMap[name] = loc;
                    }
                    locationsArray.push_back(loc);
                    while (s->getValueAddressAtIndex(test, ++loc.index) != nullptr)
                    {
                        if (s->getType() == base::STATE_SPACE_REAL_VECTOR)
                        {
                            const std::string &name = s->as<base::RealVectorStateSpace>()->getDimensionName(loc.index);
                            if (!name.empty())
                                locationsMap[name] = loc;
                        }
                        locationsArray.push_back(loc);
                    }
                }
            }
            s->freeState(test);

            if (s->isCompound())
                for (unsigned int i = 0; i < s->as<base::CompoundStateSpace>()->getSubspaceCount(); ++i)
                {
                    loc.stateLocation.chain.push_back(i);
                    computeLocationsHelper(s->as<base::CompoundStateSpace>()->getSubspace(i).get(), substateMap,
                                           locationsArray, locationsMap, loc);
                    loc.stateLocation.chain.pop_back();
                }
        }

        void computeLocationsHelper(const StateSpace *s,
                                    std::map<std::string, StateSpace::SubstateLocation> &substateMap,
                                    std::vector<StateSpace::ValueLocation> &locationsArray,
                                    std::map<std::string, StateSpace::ValueLocation> &locationsMap)
        {
            substateMap.clear();
            locationsArray.clear();
            locationsMap.clear();
            computeLocationsHelper(s, substateMap, locationsArray, locationsMap, StateSpace::ValueLocation());
        }
    }
}
/// @endcond

const std::string &ompl::base::StateSpace::getName() const
{
    return name_;
}

void ompl::base::StateSpace::setName(const std::string &name)
{
    name_ = name;

    // we don't want to call this function during the state space construction because calls to virtual functions are
    // made,
    // so we check if any values were previously inserted as value locations;
    // if none were, then we either have none (so no need to call this function again)
    // or setup() was not yet called
    if (!valueLocationsInOrder_.empty())
        computeLocationsHelper(this, substateLocationsByName_, valueLocationsInOrder_, valueLocationsByName_);
}

void ompl::base::StateSpace::computeLocations()
{
    computeLocationsHelper(this, substateLocationsByName_, valueLocationsInOrder_, valueLocationsByName_);
}

void ompl::base::StateSpace::computeSignature(std::vector<int> &signature) const
{
    signature.clear();
    computeStateSpaceSignatureHelper(this, signature);
    signature.insert(signature.begin(), signature.size());
}

ompl::base::State *ompl::base::StateSpace::cloneState(const State *source) const
{
    State *copy = allocState();
    copyState(copy, source);
    return copy;
}

void ompl::base::StateSpace::registerProjections()
{
}

void ompl::base::StateSpace::setup()
{
    maxExtent_ = getMaximumExtent();
    longestValidSegment_ = maxExtent_ * longestValidSegmentFraction_;

    if (longestValidSegment_ < std::numeric_limits<double>::epsilon())
    {
        std::stringstream error;
        error << "The longest valid segment for state space " + getName() + " must be positive." << std::endl;
        error << "Space settings:" << std::endl;
        printSettings(error);
        throw Exception(error.str());
    }

    computeLocationsHelper(this, substateLocationsByName_, valueLocationsInOrder_, valueLocationsByName_);

    // make sure we don't overwrite projections that have been configured by the user
    std::map<std::string, ProjectionEvaluatorPtr> oldProjections = projections_;
    registerProjections();
    for (auto &oldProjection : oldProjections)
        if (oldProjection.second->userConfigured())
        {
            auto o = projections_.find(oldProjection.first);
            if (o != projections_.end())
                if (!o->second->userConfigured())
                    projections_[oldProjection.first] = oldProjection.second;
        }

    // remove previously set parameters for projections
    std::vector<std::string> pnames;
    params_.getParamNames(pnames);
    for (const auto &pname : pnames)
        if (pname.substr(0, 11) == "projection.")
            params_.remove(pname);

    // setup projections and add their parameters
    for (const auto &projection : projections_)
    {
        projection.second->setup();
        if (projection.first == DEFAULT_PROJECTION_NAME)
            params_.include(projection.second->params(), "projection");
        else
            params_.include(projection.second->params(), "projection." + projection.first);
    }
}

const std::map<std::string, ompl::base::StateSpace::SubstateLocation> &
ompl::base::StateSpace::getSubstateLocationsByName() const
{
    return substateLocationsByName_;
}

ompl::base::State *ompl::base::StateSpace::getSubstateAtLocation(State *state, const SubstateLocation &loc) const
{
    std::size_t index = 0;
    while (loc.chain.size() > index)
        state = state->as<CompoundState>()->components[loc.chain[index++]];
    return state;
}

const ompl::base::State *ompl::base::StateSpace::getSubstateAtLocation(const State *state,
                                                                       const SubstateLocation &loc) const
{
    std::size_t index = 0;
    while (loc.chain.size() > index)
        state = state->as<CompoundState>()->components[loc.chain[index++]];
    return state;
}

double *ompl::base::StateSpace::getValueAddressAtIndex(State * /*state*/, const unsigned int /*index*/) const
{
    return nullptr;
}

const double *ompl::base::StateSpace::getValueAddressAtIndex(const State *state, const unsigned int index) const
{
    double *val = getValueAddressAtIndex(const_cast<State *>(state),
                                         index);  // this const-cast does not hurt, since the state is not modified
    return val;
}

const std::vector<ompl::base::StateSpace::ValueLocation> &ompl::base::StateSpace::getValueLocations() const
{
    return valueLocationsInOrder_;
}

const std::map<std::string, ompl::base::StateSpace::ValueLocation> &
ompl::base::StateSpace::getValueLocationsByName() const
{
    return valueLocationsByName_;
}

void ompl::base::StateSpace::copyToReals(std::vector<double> &reals, const State *source) const
{
    const auto &locations = getValueLocations();
    reals.resize(locations.size());
    for (std::size_t i = 0; i < locations.size(); ++i)
        reals[i] = *getValueAddressAtLocation(source, locations[i]);
}

void ompl::base::StateSpace::copyFromReals(State *destination, const std::vector<double> &reals) const
{
    const auto &locations = getValueLocations();
    assert(reals.size() == locations.size());
    for (std::size_t i = 0; i < reals.size(); ++i)
        *getValueAddressAtLocation(destination, locations[i]) = reals[i];
}

double *ompl::base::StateSpace::getValueAddressAtLocation(State *state, const ValueLocation &loc) const
{
    std::size_t index = 0;
    while (loc.stateLocation.chain.size() > index)
        state = state->as<CompoundState>()->components[loc.stateLocation.chain[index++]];
    return loc.stateLocation.space->getValueAddressAtIndex(state, loc.index);
}

const double *ompl::base::StateSpace::getValueAddressAtLocation(const State *state, const ValueLocation &loc) const
{
    std::size_t index = 0;
    while (loc.stateLocation.chain.size() > index)
        state = state->as<CompoundState>()->components[loc.stateLocation.chain[index++]];
    return loc.stateLocation.space->getValueAddressAtIndex(state, loc.index);
}

double *ompl::base::StateSpace::getValueAddressAtName(State *state, const std::string &name) const
{
    const auto &locations = getValueLocationsByName();
    auto it = locations.find(name);
    return (it != locations.end()) ? getValueAddressAtLocation(state, it->second) : nullptr;
}

const double *ompl::base::StateSpace::getValueAddressAtName(const State *state, const std::string &name) const
{
    const auto &locations = getValueLocationsByName();
    auto it = locations.find(name);
    return (it != locations.end()) ? getValueAddressAtLocation(state, it->second) : nullptr;
}

unsigned int ompl::base::StateSpace::getSerializationLength() const
{
    return 0;
}

void ompl::base::StateSpace::serialize(void * /*serialization*/, const State * /*state*/) const
{
}

void ompl::base::StateSpace::deserialize(State * /*state*/, const void * /*serialization*/) const
{
}

void ompl::base::StateSpace::printState(const State *state, std::ostream &out) const
{
    out << "State instance [" << state << ']' << std::endl;
}

void ompl::base::StateSpace::printSettings(std::ostream &out) const
{
    out << "StateSpace '" << getName() << "' instance: " << this << std::endl;
    printProjections(out);
}

void ompl::base::StateSpace::printProjections(std::ostream &out) const
{
    if (projections_.empty())
        out << "No registered projections" << std::endl;
    else
    {
        out << "Registered projections:" << std::endl;
        for (const auto &projection : projections_)
        {
            out << "  - ";
            if (projection.first == DEFAULT_PROJECTION_NAME)
                out << "<default>";
            else
                out << projection.first;
            out << std::endl;
            projection.second->printSettings(out);
        }
    }
}

/// @cond IGNORE
namespace ompl
{
    namespace base
    {
        static bool StateSpaceIncludes(const StateSpace *self, const StateSpace *other)
        {
            std::queue<const StateSpace *> q;
            q.push(self);
            while (!q.empty())
            {
                const StateSpace *m = q.front();
                q.pop();
                if (m->getName() == other->getName())
                    return true;
                if (m->isCompound())
                {
                    unsigned int c = m->as<CompoundStateSpace>()->getSubspaceCount();
                    for (unsigned int i = 0; i < c; ++i)
                        q.push(m->as<CompoundStateSpace>()->getSubspace(i).get());
                }
            }
            return false;
        }

        static bool StateSpaceCovers(const StateSpace *self, const StateSpace *other)
        {
            if (StateSpaceIncludes(self, other))
                return true;
            else if (other->isCompound())
            {
                unsigned int c = other->as<CompoundStateSpace>()->getSubspaceCount();
                for (unsigned int i = 0; i < c; ++i)
                    if (!StateSpaceCovers(self, other->as<CompoundStateSpace>()->getSubspace(i).get()))
                        return false;
                return true;
            }
            return false;
        }

        struct CompareSubstateLocation
        {
            bool operator()(const StateSpace::SubstateLocation &a, const StateSpace::SubstateLocation &b) const
            {
                if (a.space->getDimension() != b.space->getDimension())
                    return a.space->getDimension() > b.space->getDimension();
                return a.space->getName() > b.space->getName();
            }
        };
    }
}

/// @endcond

bool ompl::base::StateSpace::covers(const StateSpacePtr &other) const
{
    return StateSpaceCovers(this, other.get());
}

bool ompl::base::StateSpace::includes(const StateSpacePtr &other) const
{
    return StateSpaceIncludes(this, other.get());
}

bool ompl::base::StateSpace::covers(const StateSpace *other) const
{
    return StateSpaceCovers(this, other);
}

bool ompl::base::StateSpace::includes(const StateSpace *other) const
{
    return StateSpaceIncludes(this, other);
}

void ompl::base::StateSpace::getCommonSubspaces(const StateSpacePtr &other, std::vector<std::string> &subspaces) const
{
    getCommonSubspaces(other.get(), subspaces);
}

void ompl::base::StateSpace::getCommonSubspaces(const StateSpace *other, std::vector<std::string> &subspaces) const
{
    std::set<StateSpace::SubstateLocation, CompareSubstateLocation> intersection;
    const std::map<std::string, StateSpace::SubstateLocation> &S = other->getSubstateLocationsByName();
    for (const auto &it : substateLocationsByName_)
    {
        if (S.find(it.first) != S.end())
            intersection.insert(it.second);
    }

    bool found = true;
    while (found)
    {
        found = false;
        for (auto it = intersection.begin(); it != intersection.end(); ++it)
            for (auto jt = intersection.begin(); jt != intersection.end(); ++jt)
                if (it != jt)
                    if (StateSpaceCovers(it->space, jt->space))
                    {
                        intersection.erase(jt);
                        found = true;
                        break;
                    }
    }
    subspaces.clear();
    for (const auto &it : intersection)
        subspaces.push_back(it.space->getName());
}

void ompl::base::StateSpace::List(std::ostream &out)
{
    AllocatedSpaces &as = getAllocatedSpaces();
    std::lock_guard<std::mutex> smLock(as.lock_);
    for (auto &it : as.list_)
        out << "@ " << it << ": " << it->getName() << std::endl;
}

void ompl::base::StateSpace::list(std::ostream &out) const
{
    std::queue<const StateSpace *> q;
    q.push(this);
    while (!q.empty())
    {
        const StateSpace *m = q.front();
        q.pop();
        out << "@ " << m << ": " << m->getName() << std::endl;
        if (m->isCompound())
        {
            unsigned int c = m->as<CompoundStateSpace>()->getSubspaceCount();
            for (unsigned int i = 0; i < c; ++i)
                q.push(m->as<CompoundStateSpace>()->getSubspace(i).get());
        }
    }
}

void ompl::base::StateSpace::diagram(std::ostream &out) const
{
    out << "digraph StateSpace {" << std::endl;
    out << '"' << getName() << '"' << std::endl;

    std::queue<const StateSpace *> q;
    q.push(this);
    while (!q.empty())
    {
        const StateSpace *m = q.front();
        q.pop();
        if (m->isCompound())
        {
            unsigned int c = m->as<CompoundStateSpace>()->getSubspaceCount();
            for (unsigned int i = 0; i < c; ++i)
            {
                const StateSpace *s = m->as<CompoundStateSpace>()->getSubspace(i).get();
                q.push(s);
                out << '"' << m->getName() << R"(" -> ")" << s->getName() << R"(" [label=")"
                    << ompl::toString(m->as<CompoundStateSpace>()->getSubspaceWeight(i)) << R"("];)" << std::endl;
            }
        }
    }

    out << '}' << std::endl;
}

void ompl::base::StateSpace::Diagram(std::ostream &out)
{
    AllocatedSpaces &as = getAllocatedSpaces();
    std::lock_guard<std::mutex> smLock(as.lock_);
    out << "digraph StateSpaces {" << std::endl;
    for (auto it = as.list_.begin(); it != as.list_.end(); ++it)
    {
        out << '"' << (*it)->getName() << '"' << std::endl;
        for (auto jt = as.list_.begin(); jt != as.list_.end(); ++jt)
            if (it != jt)
            {
                if ((*it)->isCompound() && (*it)->as<CompoundStateSpace>()->hasSubspace((*jt)->getName()))
                    out << '"' << (*it)->getName() << R"(" -> ")" << (*jt)->getName() << R"(" [label=")"
                        << ompl::toString((*it)->as<CompoundStateSpace>()->getSubspaceWeight((*jt)->getName())) <<
                        R"("];)" << std::endl;
                else if (!StateSpaceIncludes(*it, *jt) && StateSpaceCovers(*it, *jt))
                    out << '"' << (*it)->getName() << R"(" -> ")" << (*jt)->getName() << R"(" [style=dashed];)"
                        << std::endl;
            }
    }
    out << '}' << std::endl;
}

void ompl::base::StateSpace::sanityChecks() const
{
    unsigned int flags = isMetricSpace() ? ~0 : ~(STATESPACE_DISTANCE_SYMMETRIC | STATESPACE_TRIANGLE_INEQUALITY);
    sanityChecks(std::numeric_limits<double>::epsilon(), std::numeric_limits<float>::epsilon(), flags);
}

void ompl::base::StateSpace::sanityChecks(double zero, double eps, unsigned int flags) const
{
    {
        double maxExt = getMaximumExtent();

        State *s1 = allocState();
        State *s2 = allocState();
        StateSamplerPtr ss = allocStateSampler();
        char *serialization = nullptr;
        if ((flags & STATESPACE_SERIALIZATION) && getSerializationLength() > 0)
            serialization = new char[getSerializationLength()];
        for (unsigned int i = 0; i < magic::TEST_STATE_COUNT; ++i)
        {
            ss->sampleUniform(s1);
            if (distance(s1, s1) > eps)
                throw Exception("Distance from a state to itself should be 0");
            if (!equalStates(s1, s1))
                throw Exception("A state should be equal to itself");
            if ((flags & STATESPACE_RESPECT_BOUNDS) && !satisfiesBounds(s1))
                throw Exception("Sampled states should be within bounds");
            copyState(s2, s1);
            if (!equalStates(s1, s2))
                throw Exception("Copy of a state is not the same as the original state. copyState() may not work "
                                "correctly.");
            if (flags & STATESPACE_ENFORCE_BOUNDS_NO_OP)
            {
                enforceBounds(s1);
                if (!equalStates(s1, s2))
                    throw Exception("enforceBounds() seems to modify states that are in fact within bounds.");
            }
            if (flags & STATESPACE_SERIALIZATION)
            {
                ss->sampleUniform(s2);
                serialize(serialization, s1);
                deserialize(s2, serialization);
                if (!equalStates(s1, s2))
                    throw Exception("Serialization/deserialization operations do not seem to work as expected.");
            }
            ss->sampleUniform(s2);
            if (!equalStates(s1, s2))
            {
                double d12 = distance(s1, s2);
                if ((flags & STATESPACE_DISTANCE_DIFFERENT_STATES) && d12 < zero)
                    throw Exception("Distance between different states should be above 0");
                double d21 = distance(s2, s1);
                if ((flags & STATESPACE_DISTANCE_SYMMETRIC) && fabs(d12 - d21) > eps)
                    throw Exception("The distance function should be symmetric (A->B=" + ompl::toString(d12) +
                                    ", B->A=" + ompl::toString(d21) + ", difference is " +
                                    ompl::toString(fabs(d12 - d21)) + ")");
                if (flags & STATESPACE_DISTANCE_BOUND)
                    if (d12 > maxExt + zero)
                        throw Exception("The distance function should not report values larger than the maximum extent "
                                        "(" +
                                        ompl::toString(d12) + " > " + ompl::toString(maxExt) + ")");
            }
        }
        if (serialization)
            delete[] serialization;
        freeState(s1);
        freeState(s2);
    }

    // Test that interpolation works as expected and also test triangle inequality
    if (!isDiscrete() && !isHybrid() && (flags & (STATESPACE_INTERPOLATION | STATESPACE_TRIANGLE_INEQUALITY)))
    {
        State *s1 = allocState();
        State *s2 = allocState();
        State *s3 = allocState();
        StateSamplerPtr ss = allocStateSampler();

        for (unsigned int i = 0; i < magic::TEST_STATE_COUNT; ++i)
        {
            ss->sampleUniform(s1);
            ss->sampleUniform(s2);
            ss->sampleUniform(s3);

            interpolate(s1, s2, 0.0, s3);
            if ((flags & STATESPACE_INTERPOLATION) && distance(s1, s3) > eps)
                throw Exception("Interpolation from a state at time 0 should be not change the original state");

            interpolate(s1, s2, 1.0, s3);
            if ((flags & STATESPACE_INTERPOLATION) && distance(s2, s3) > eps)
                throw Exception("Interpolation to a state at time 1 should be the same as the final state");

            interpolate(s1, s2, 0.5, s3);
            double diff = distance(s1, s3) + distance(s3, s2) - distance(s1, s2);
            if ((flags & STATESPACE_TRIANGLE_INEQUALITY) && diff < -eps)
                throw Exception("Interpolation to midpoint state does not lead to distances that satisfy the triangle "
                                "inequality (" +
                                ompl::toString(diff) + " difference)");

            interpolate(s3, s2, 0.5, s3);
            interpolate(s1, s2, 0.75, s2);

            if ((flags & STATESPACE_INTERPOLATION) && distance(s2, s3) > eps)
                throw Exception("Continued interpolation does not work as expected. Please also check that "
                                "interpolate() works with overlapping memory for its state arguments");
        }
        freeState(s1);
        freeState(s2);
        freeState(s3);
    }
}

bool ompl::base::StateSpace::hasDefaultProjection() const
{
    return hasProjection(DEFAULT_PROJECTION_NAME);
}

bool ompl::base::StateSpace::hasProjection(const std::string &name) const
{
    return projections_.find(name) != projections_.end();
}

ompl::base::ProjectionEvaluatorPtr ompl::base::StateSpace::getDefaultProjection() const
{
    if (hasDefaultProjection())
        return getProjection(DEFAULT_PROJECTION_NAME);
    else
    {
        OMPL_ERROR("No default projection is set. Perhaps setup() needs to be called");
        return ProjectionEvaluatorPtr();
    }
}

ompl::base::ProjectionEvaluatorPtr ompl::base::StateSpace::getProjection(const std::string &name) const
{
    auto it = projections_.find(name);
    if (it != projections_.end())
        return it->second;
    else
    {
        OMPL_ERROR("Projection '%s' is not defined", name.c_str());
        return ProjectionEvaluatorPtr();
    }
}

const std::map<std::string, ompl::base::ProjectionEvaluatorPtr> &
ompl::base::StateSpace::getRegisteredProjections() const
{
    return projections_;
}

void ompl::base::StateSpace::registerDefaultProjection(const ProjectionEvaluatorPtr &projection)
{
    registerProjection(DEFAULT_PROJECTION_NAME, projection);
}

void ompl::base::StateSpace::registerProjection(const std::string &name, const ProjectionEvaluatorPtr &projection)
{
    if (projection)
        projections_[name] = projection;
    else
        OMPL_ERROR("Attempting to register invalid projection under name '%s'. Ignoring.", name.c_str());
}

bool ompl::base::StateSpace::isCompound() const
{
    return false;
}

bool ompl::base::StateSpace::isDiscrete() const
{
    return false;
}

bool ompl::base::StateSpace::isHybrid() const
{
    return false;
}

bool ompl::base::StateSpace::hasSymmetricDistance() const
{
    return true;
}

bool ompl::base::StateSpace::hasSymmetricInterpolate() const
{
    return true;
}

void ompl::base::StateSpace::setStateSamplerAllocator(const StateSamplerAllocator &ssa)
{
    ssa_ = ssa;
}

void ompl::base::StateSpace::clearStateSamplerAllocator()
{
    ssa_ = StateSamplerAllocator();
}

ompl::base::StateSamplerPtr ompl::base::StateSpace::allocStateSampler() const
{
    if (ssa_)
        return ssa_(this);
    else
        return allocDefaultStateSampler();
}

ompl::base::StateSamplerPtr ompl::base::StateSpace::allocSubspaceStateSampler(const StateSpacePtr &subspace) const
{
    return allocSubspaceStateSampler(subspace.get());
}

ompl::base::StateSamplerPtr ompl::base::StateSpace::allocSubspaceStateSampler(const StateSpace *subspace) const
{
    if (subspace->getName() == getName())
        return allocStateSampler();
    return std::make_shared<SubspaceStateSampler>(this, subspace, 1.0);
}

void ompl::base::StateSpace::setValidSegmentCountFactor(unsigned int factor)
{
    if (factor < 1)
        throw Exception("The multiplicative factor for the valid segment count between two states must be strictly "
                        "positive");
    longestValidSegmentCountFactor_ = factor;
}

void ompl::base::StateSpace::setLongestValidSegmentFraction(double segmentFraction)
{
    if (segmentFraction < std::numeric_limits<double>::epsilon() ||
        segmentFraction > 1.0 - std::numeric_limits<double>::epsilon())
        throw Exception("The fraction of the extent must be larger than 0 and less than 1");
    longestValidSegmentFraction_ = segmentFraction;
}

unsigned int ompl::base::StateSpace::getValidSegmentCountFactor() const
{
    return longestValidSegmentCountFactor_;
}

double ompl::base::StateSpace::getLongestValidSegmentFraction() const
{
    return longestValidSegmentFraction_;
}

double ompl::base::StateSpace::getLongestValidSegmentLength() const
{
    return longestValidSegment_;
}

unsigned int ompl::base::StateSpace::validSegmentCount(const State *state1, const State *state2) const
{
    return longestValidSegmentCountFactor_ * (unsigned int)ceil(distance(state1, state2) / longestValidSegment_);
}

ompl::base::CompoundStateSpace::CompoundStateSpace()
{
    setName("Compound" + getName());
}

ompl::base::CompoundStateSpace::CompoundStateSpace(const std::vector<StateSpacePtr> &components,
                                                   const std::vector<double> &weights)
{
    if (components.size() != weights.size())
        throw Exception("Number of component spaces and weights are not the same");
    setName("Compound" + getName());
    for (unsigned int i = 0; i < components.size(); ++i)
        addSubspace(components[i], weights[i]);
}

void ompl::base::CompoundStateSpace::addSubspace(const StateSpacePtr &component, double weight)
{
    if (locked_)
        throw Exception("This state space is locked. No further components can be added");
    if (weight < 0.0)
        throw Exception("Subspace weight cannot be negative");
    components_.push_back(component);
    weights_.push_back(weight);
    weightSum_ += weight;
    componentCount_ = components_.size();
}

bool ompl::base::CompoundStateSpace::isCompound() const
{
    return true;
}

bool ompl::base::CompoundStateSpace::isHybrid() const
{
    bool c = false;
    bool d = false;
    for (const auto &component : components_)
    {
        if (component->isHybrid())
            return true;
        if (component->isDiscrete())
            d = true;
        else
            c = true;
    }
    return c && d;
}

unsigned int ompl::base::CompoundStateSpace::getSubspaceCount() const
{
    return componentCount_;
}

const ompl::base::StateSpacePtr &ompl::base::CompoundStateSpace::getSubspace(const unsigned int index) const
{
    if (componentCount_ > index)
        return components_[index];
    else
        throw Exception("Subspace index does not exist");
}

bool ompl::base::CompoundStateSpace::hasSubspace(const std::string &name) const
{
    for (const auto &component : components_)
        if (component->getName() == name)
            return true;
    return false;
}

unsigned int ompl::base::CompoundStateSpace::getSubspaceIndex(const std::string &name) const
{
    for (unsigned int i = 0; i < componentCount_; ++i)
        if (components_[i]->getName() == name)
            return i;
    throw Exception("Subspace " + name + " does not exist");
}

const ompl::base::StateSpacePtr &ompl::base::CompoundStateSpace::getSubspace(const std::string &name) const
{
    return components_[getSubspaceIndex(name)];
}

double ompl::base::CompoundStateSpace::getSubspaceWeight(const unsigned int index) const
{
    if (componentCount_ > index)
        return weights_[index];
    else
        throw Exception("Subspace index does not exist");
}

double ompl::base::CompoundStateSpace::getSubspaceWeight(const std::string &name) const
{
    for (unsigned int i = 0; i < componentCount_; ++i)
        if (components_[i]->getName() == name)
            return weights_[i];
    throw Exception("Subspace " + name + " does not exist");
}

void ompl::base::CompoundStateSpace::setSubspaceWeight(const unsigned int index, double weight)
{
    if (weight < 0.0)
        throw Exception("Subspace weight cannot be negative");
    if (componentCount_ > index)
    {
        weightSum_ += weight - weights_[index];
        weights_[index] = weight;
    }
    else
        throw Exception("Subspace index does not exist");
}

void ompl::base::CompoundStateSpace::setSubspaceWeight(const std::string &name, double weight)
{
    for (unsigned int i = 0; i < componentCount_; ++i)
        if (components_[i]->getName() == name)
        {
            setSubspaceWeight(i, weight);
            return;
        }
    throw Exception("Subspace " + name + " does not exist");
}

const std::vector<ompl::base::StateSpacePtr> &ompl::base::CompoundStateSpace::getSubspaces() const
{
    return components_;
}

const std::vector<double> &ompl::base::CompoundStateSpace::getSubspaceWeights() const
{
    return weights_;
}

unsigned int ompl::base::CompoundStateSpace::getDimension() const
{
    unsigned int dim = 0;
    for (unsigned int i = 0; i < componentCount_; ++i)
        dim += components_[i]->getDimension();
    return dim;
}

double ompl::base::CompoundStateSpace::getMaximumExtent() const
{
    double e = 0.0;
    for (unsigned int i = 0; i < componentCount_; ++i)
        if (weights_[i] >= std::numeric_limits<double>::epsilon())  // avoid possible multiplication of 0 times infinity
            e += weights_[i] * components_[i]->getMaximumExtent();
    return e;
}

double ompl::base::CompoundStateSpace::getMeasure() const
{
    double m = 1.0;
    for (unsigned int i = 0; i < componentCount_; ++i)
        if (weights_[i] >= std::numeric_limits<double>::epsilon())  // avoid possible multiplication of 0 times infinity
            m *= weights_[i] * components_[i]->getMeasure();
    return m;
}

void ompl::base::CompoundStateSpace::enforceBounds(State *state) const
{
    auto *cstate = static_cast<CompoundState *>(state);
    for (unsigned int i = 0; i < componentCount_; ++i)
        components_[i]->enforceBounds(cstate->components[i]);
}

bool ompl::base::CompoundStateSpace::satisfiesBounds(const State *state) const
{
    const auto *cstate = static_cast<const CompoundState *>(state);
    for (unsigned int i = 0; i < componentCount_; ++i)
        if (!components_[i]->satisfiesBounds(cstate->components[i]))
            return false;
    return true;
}

void ompl::base::CompoundStateSpace::copyState(State *destination, const State *source) const
{
    auto *cdest = static_cast<CompoundState *>(destination);
    const auto *csrc = static_cast<const CompoundState *>(source);
    for (unsigned int i = 0; i < componentCount_; ++i)
        components_[i]->copyState(cdest->components[i], csrc->components[i]);
}

unsigned int ompl::base::CompoundStateSpace::getSerializationLength() const
{
    unsigned int l = 0;
    for (const auto &component : components_)
        l += component->getSerializationLength();
    return l;
}

void ompl::base::CompoundStateSpace::serialize(void *serialization, const State *state) const
{
    const auto *cstate = static_cast<const CompoundState *>(state);
    unsigned int l = 0;
    for (unsigned int i = 0; i < componentCount_; ++i)
    {
        components_[i]->serialize(reinterpret_cast<char *>(serialization) + l, cstate->components[i]);
        l += components_[i]->getSerializationLength();
    }
}

void ompl::base::CompoundStateSpace::deserialize(State *state, const void *serialization) const
{
    auto *cstate = static_cast<CompoundState *>(state);
    unsigned int l = 0;
    for (unsigned int i = 0; i < componentCount_; ++i)
    {
        components_[i]->deserialize(cstate->components[i], reinterpret_cast<const char *>(serialization) + l);
        l += components_[i]->getSerializationLength();
    }
}

double ompl::base::CompoundStateSpace::distance(const State *state1, const State *state2) const
{
    const auto *cstate1 = static_cast<const CompoundState *>(state1);
    const auto *cstate2 = static_cast<const CompoundState *>(state2);
    double dist = 0.0;
    for (unsigned int i = 0; i < componentCount_; ++i)
        dist += weights_[i] * components_[i]->distance(cstate1->components[i], cstate2->components[i]);
    return dist;
}

void ompl::base::CompoundStateSpace::setLongestValidSegmentFraction(double segmentFraction)
{
    StateSpace::setLongestValidSegmentFraction(segmentFraction);
    for (const auto &component : components_)
        component->setLongestValidSegmentFraction(segmentFraction);
}

unsigned int ompl::base::CompoundStateSpace::validSegmentCount(const State *state1, const State *state2) const
{
    const auto *cstate1 = static_cast<const CompoundState *>(state1);
    const auto *cstate2 = static_cast<const CompoundState *>(state2);
    unsigned int sc = 0;
    for (unsigned int i = 0; i < componentCount_; ++i)
    {
        unsigned int sci = components_[i]->validSegmentCount(cstate1->components[i], cstate2->components[i]);
        if (sci > sc)
            sc = sci;
    }
    return sc;
}

bool ompl::base::CompoundStateSpace::equalStates(const State *state1, const State *state2) const
{
    const auto *cstate1 = static_cast<const CompoundState *>(state1);
    const auto *cstate2 = static_cast<const CompoundState *>(state2);
    for (unsigned int i = 0; i < componentCount_; ++i)
        if (!components_[i]->equalStates(cstate1->components[i], cstate2->components[i]))
            return false;
    return true;
}

void ompl::base::CompoundStateSpace::interpolate(const State *from, const State *to, const double t, State *state) const
{
    const auto *cfrom = static_cast<const CompoundState *>(from);
    const auto *cto = static_cast<const CompoundState *>(to);
    auto *cstate = static_cast<CompoundState *>(state);
    for (unsigned int i = 0; i < componentCount_; ++i)
        components_[i]->interpolate(cfrom->components[i], cto->components[i], t, cstate->components[i]);
}

ompl::base::StateSamplerPtr ompl::base::CompoundStateSpace::allocDefaultStateSampler() const
{
    auto ss(std::make_shared<CompoundStateSampler>(this));
    if (weightSum_ < std::numeric_limits<double>::epsilon())
        for (unsigned int i = 0; i < componentCount_; ++i)
            ss->addSampler(components_[i]->allocStateSampler(), 1.0);
    else
        for (unsigned int i = 0; i < componentCount_; ++i)
            ss->addSampler(components_[i]->allocStateSampler(), weights_[i] / weightSum_);
    return ss;
}

ompl::base::StateSamplerPtr ompl::base::CompoundStateSpace::allocSubspaceStateSampler(const StateSpace *subspace) const
{
    if (subspace->getName() == getName())
        return allocStateSampler();
    if (hasSubspace(subspace->getName()))
        return std::make_shared<SubspaceStateSampler>(this, subspace,
                                                      getSubspaceWeight(subspace->getName()) / weightSum_);
    return StateSpace::allocSubspaceStateSampler(subspace);
}

ompl::base::State *ompl::base::CompoundStateSpace::allocState() const
{
    auto *state = new CompoundState();
    allocStateComponents(state);
    return static_cast<State *>(state);
}

void ompl::base::CompoundStateSpace::allocStateComponents(CompoundState *state) const
{
    state->components = new State *[componentCount_];
    for (unsigned int i = 0; i < componentCount_; ++i)
        state->components[i] = components_[i]->allocState();
}

void ompl::base::CompoundStateSpace::freeState(State *state) const
{
    auto *cstate = static_cast<CompoundState *>(state);
    for (unsigned int i = 0; i < componentCount_; ++i)
        components_[i]->freeState(cstate->components[i]);
    delete[] cstate->components;
    delete cstate;
}

void ompl::base::CompoundStateSpace::lock()
{
    locked_ = true;
}

bool ompl::base::CompoundStateSpace::isLocked() const
{
    return locked_;
}

double *ompl::base::CompoundStateSpace::getValueAddressAtIndex(State *state, const unsigned int index) const
{
    auto *cstate = static_cast<CompoundState *>(state);
    unsigned int idx = 0;

    for (unsigned int i = 0; i < componentCount_; ++i)
        for (unsigned int j = 0; j <= index; ++j)
        {
            double *va = components_[i]->getValueAddressAtIndex(cstate->components[i], j);
            if (va)
            {
                if (idx == index)
                    return va;
                else
                    idx++;
            }
            else
                break;
        }
    return nullptr;
}

void ompl::base::CompoundStateSpace::printState(const State *state, std::ostream &out) const
{
    out << "Compound state [" << std::endl;
    const auto *cstate = static_cast<const CompoundState *>(state);
    for (unsigned int i = 0; i < componentCount_; ++i)
        components_[i]->printState(cstate->components[i], out);
    out << "]" << std::endl;
}

void ompl::base::CompoundStateSpace::printSettings(std::ostream &out) const
{
    out << "Compound state space '" << getName() << "' of dimension " << getDimension()
        << (isLocked() ? " (locked)" : "") << " [" << std::endl;
    for (unsigned int i = 0; i < componentCount_; ++i)
    {
        components_[i]->printSettings(out);
        out << " of weight " << weights_[i] << std::endl;
    }
    out << "]" << std::endl;
    printProjections(out);
}

void ompl::base::CompoundStateSpace::setup()
{
    for (const auto &component : components_)
        component->setup();

    StateSpace::setup();
}

void ompl::base::CompoundStateSpace::computeLocations()
{
    StateSpace::computeLocations();
    for (const auto &component : components_)
        component->computeLocations();
}

namespace ompl
{
    namespace base
    {
        AdvancedStateCopyOperation copyStateData(const StateSpacePtr &destS, State *dest, const StateSpacePtr &sourceS,
                                                 const State *source)
        {
            return copyStateData(destS.get(), dest, sourceS.get(), source);
        }

        AdvancedStateCopyOperation copyStateData(const StateSpace *destS, State *dest, const StateSpace *sourceS,
                                                 const State *source)
        {
            // if states correspond to the same space, simply do copy
            if (destS->getName() == sourceS->getName())
            {
                if (dest != source)
                    destS->copyState(dest, source);
                return ALL_DATA_COPIED;
            }

            AdvancedStateCopyOperation result = NO_DATA_COPIED;

            // if "to" state is compound
            if (destS->isCompound())
            {
                const auto *compoundDestS = destS->as<CompoundStateSpace>();
                auto *compoundDest = dest->as<CompoundState>();

                // if there is a subspace in "to" that corresponds to "from", set the data and return
                for (unsigned int i = 0; i < compoundDestS->getSubspaceCount(); ++i)
                    if (compoundDestS->getSubspace(i)->getName() == sourceS->getName())
                    {
                        if (compoundDest->components[i] != source)
                            compoundDestS->getSubspace(i)->copyState(compoundDest->components[i], source);
                        return ALL_DATA_COPIED;
                    }

                // it could be there are further levels of compound spaces where the data can be set
                // so we call this function recursively
                for (unsigned int i = 0; i < compoundDestS->getSubspaceCount(); ++i)
                {
                    AdvancedStateCopyOperation res = copyStateData(compoundDestS->getSubspace(i).get(),
                                                                   compoundDest->components[i], sourceS, source);

                    if (res != NO_DATA_COPIED)
                        result = SOME_DATA_COPIED;

                    // if all data was copied, we stop
                    if (res == ALL_DATA_COPIED)
                        return ALL_DATA_COPIED;
                }
            }

            // if we got to this point, it means that the data in "from" could not be copied as a chunk to "to"
            // it could be the case "from" is from a compound space as well, so we can copy parts of "from", as needed
            if (sourceS->isCompound())
            {
                const auto *compoundSourceS = sourceS->as<CompoundStateSpace>();
                const auto *compoundSource = source->as<CompoundState>();

                unsigned int copiedComponents = 0;

                // if there is a subspace in "to" that corresponds to "from", set the data and return
                for (unsigned int i = 0; i < compoundSourceS->getSubspaceCount(); ++i)
                {
                    AdvancedStateCopyOperation res = copyStateData(destS, dest, compoundSourceS->getSubspace(i).get(),
                                                                   compoundSource->components[i]);
                    if (res == ALL_DATA_COPIED)
                        copiedComponents++;
                    if (res != NO_DATA_COPIED)
                        result = SOME_DATA_COPIED;
                }

                // if each individual component got copied, then the entire data in "from" got copied
                if (copiedComponents == compoundSourceS->getSubspaceCount())
                    result = ALL_DATA_COPIED;
            }

            return result;
        }

        AdvancedStateCopyOperation copyStateData(const StateSpacePtr &destS, State *dest, const StateSpacePtr &sourceS,
                                                 const State *source, const std::vector<std::string> &subspaces)
        {
            return copyStateData(destS.get(), dest, sourceS.get(), source, subspaces);
        }

        AdvancedStateCopyOperation copyStateData(const StateSpace *destS, State *dest, const StateSpace *sourceS,
                                                 const State *source, const std::vector<std::string> &subspaces)
        {
            std::size_t copyCount = 0;
            const std::map<std::string, StateSpace::SubstateLocation> &destLoc = destS->getSubstateLocationsByName();
            const std::map<std::string, StateSpace::SubstateLocation> &sourceLoc =
                sourceS->getSubstateLocationsByName();
            for (const auto &subspace : subspaces)
            {
                auto dt = destLoc.find(subspace);
                if (dt != destLoc.end())
                {
                    auto st = sourceLoc.find(subspace);
                    if (st != sourceLoc.end())
                    {
                        dt->second.space->copyState(destS->getSubstateAtLocation(dest, dt->second),
                                                    sourceS->getSubstateAtLocation(source, st->second));
                        ++copyCount;
                    }
                }
            }
            if (copyCount == subspaces.size())
                return ALL_DATA_COPIED;
            if (copyCount > 0)
                return SOME_DATA_COPIED;
            return NO_DATA_COPIED;
        }

        /// @cond IGNORE
        inline bool StateSpaceHasContent(const StateSpacePtr &m)
        {
            if (!m)
                return false;
            if (m->getDimension() == 0 && m->getType() == STATE_SPACE_UNKNOWN && m->isCompound())
            {
                const unsigned int nc = m->as<CompoundStateSpace>()->getSubspaceCount();
                for (unsigned int i = 0; i < nc; ++i)
                    if (StateSpaceHasContent(m->as<CompoundStateSpace>()->getSubspace(i)))
                        return true;
                return false;
            }
            return true;
        }
        /// @endcond

        StateSpacePtr operator+(const StateSpacePtr &a, const StateSpacePtr &b)
        {
            if (!StateSpaceHasContent(a) && StateSpaceHasContent(b))
                return b;

            if (!StateSpaceHasContent(b) && StateSpaceHasContent(a))
                return a;

            std::vector<StateSpacePtr> components;
            std::vector<double> weights;

            bool change = false;
            if (a)
            {
                bool used = false;
                if (auto *csm_a = dynamic_cast<CompoundStateSpace *>(a.get()))
                    if (!csm_a->isLocked())
                    {
                        used = true;
                        for (unsigned int i = 0; i < csm_a->getSubspaceCount(); ++i)
                        {
                            components.push_back(csm_a->getSubspace(i));
                            weights.push_back(csm_a->getSubspaceWeight(i));
                        }
                    }

                if (!used)
                {
                    components.push_back(a);
                    weights.push_back(1.0);
                }
            }
            if (b)
            {
                bool used = false;
                unsigned int size = components.size();

                if (auto *csm_b = dynamic_cast<CompoundStateSpace *>(b.get()))
                    if (!csm_b->isLocked())
                    {
                        used = true;
                        for (unsigned int i = 0; i < csm_b->getSubspaceCount(); ++i)
                        {
                            bool ok = true;
                            for (unsigned int j = 0; j < size; ++j)
                                if (components[j]->getName() == csm_b->getSubspace(i)->getName())
                                {
                                    ok = false;
                                    break;
                                }
                            if (ok)
                            {
                                components.push_back(csm_b->getSubspace(i));
                                weights.push_back(csm_b->getSubspaceWeight(i));
                                change = true;
                            }
                        }
                        if (components.size() == csm_b->getSubspaceCount())
                            return b;
                    }

                if (!used)
                {
                    bool ok = true;
                    for (unsigned int j = 0; j < size; ++j)
                        if (components[j]->getName() == b->getName())
                        {
                            ok = false;
                            break;
                        }
                    if (ok)
                    {
                        components.push_back(b);
                        weights.push_back(1.0);
                        change = true;
                    }
                }
            }

            if (!change && a)
                return a;

            if (components.size() == 1)
                return components[0];

            return std::make_shared<CompoundStateSpace>(components, weights);
        }

        StateSpacePtr operator-(const StateSpacePtr &a, const StateSpacePtr &b)
        {
            std::vector<StateSpacePtr> components_a;
            std::vector<double> weights_a;
            std::vector<StateSpacePtr> components_b;

            if (a)
            {
                bool used = false;
                if (auto *csm_a = dynamic_cast<CompoundStateSpace *>(a.get()))
                    if (!csm_a->isLocked())
                    {
                        used = true;
                        for (unsigned int i = 0; i < csm_a->getSubspaceCount(); ++i)
                        {
                            components_a.push_back(csm_a->getSubspace(i));
                            weights_a.push_back(csm_a->getSubspaceWeight(i));
                        }
                    }

                if (!used)
                {
                    components_a.push_back(a);
                    weights_a.push_back(1.0);
                }
            }

            if (b)
            {
                bool used = false;
                if (auto *csm_b = dynamic_cast<CompoundStateSpace *>(b.get()))
                    if (!csm_b->isLocked())
                    {
                        used = true;
                        for (unsigned int i = 0; i < csm_b->getSubspaceCount(); ++i)
                            components_b.push_back(csm_b->getSubspace(i));
                    }
                if (!used)
                    components_b.push_back(b);
            }

            bool change = false;
            for (auto &i : components_b)
                for (unsigned int j = 0; j < components_a.size(); ++j)
                    if (components_a[j]->getName() == i->getName())
                    {
                        components_a.erase(components_a.begin() + j);
                        weights_a.erase(weights_a.begin() + j);
                        change = true;
                        break;
                    }

            if (!change && a)
                return a;

            if (components_a.size() == 1)
                return components_a[0];

            return std::make_shared<CompoundStateSpace>(components_a, weights_a);
        }

        StateSpacePtr operator-(const StateSpacePtr &a, const std::string &name)
        {
            std::vector<StateSpacePtr> components;
            std::vector<double> weights;

            bool change = false;
            if (a)
            {
                bool used = false;
                if (auto *csm_a = dynamic_cast<CompoundStateSpace *>(a.get()))
                    if (!csm_a->isLocked())
                    {
                        used = true;
                        for (unsigned int i = 0; i < csm_a->getSubspaceCount(); ++i)
                        {
                            if (csm_a->getSubspace(i)->getName() == name)
                            {
                                change = true;
                                continue;
                            }
                            components.push_back(csm_a->getSubspace(i));
                            weights.push_back(csm_a->getSubspaceWeight(i));
                        }
                    }

                if (!used)
                {
                    if (a->getName() != name)
                    {
                        components.push_back(a);
                        weights.push_back(1.0);
                    }
                    else
                        change = true;
                }
            }

            if (!change && a)
                return a;

            if (components.size() == 1)
                return components[0];

            return std::make_shared<CompoundStateSpace>(components, weights);
        }

        StateSpacePtr operator*(const StateSpacePtr &a, const StateSpacePtr &b)
        {
            std::vector<StateSpacePtr> components_a;
            std::vector<double> weights_a;
            std::vector<StateSpacePtr> components_b;
            std::vector<double> weights_b;

            if (a)
            {
                bool used = false;
                if (auto *csm_a = dynamic_cast<CompoundStateSpace *>(a.get()))
                    if (!csm_a->isLocked())
                    {
                        used = true;
                        for (unsigned int i = 0; i < csm_a->getSubspaceCount(); ++i)
                        {
                            components_a.push_back(csm_a->getSubspace(i));
                            weights_a.push_back(csm_a->getSubspaceWeight(i));
                        }
                    }

                if (!used)
                {
                    components_a.push_back(a);
                    weights_a.push_back(1.0);
                }
            }

            if (b)
            {
                bool used = false;
                if (auto *csm_b = dynamic_cast<CompoundStateSpace *>(b.get()))
                    if (!csm_b->isLocked())
                    {
                        used = true;
                        for (unsigned int i = 0; i < csm_b->getSubspaceCount(); ++i)
                        {
                            components_b.push_back(csm_b->getSubspace(i));
                            weights_b.push_back(csm_b->getSubspaceWeight(i));
                        }
                    }

                if (!used)
                {
                    components_b.push_back(b);
                    weights_b.push_back(1.0);
                }
            }

            std::vector<StateSpacePtr> components;
            std::vector<double> weights;

            for (unsigned int i = 0; i < components_b.size(); ++i)
            {
                for (unsigned int j = 0; j < components_a.size(); ++j)
                    if (components_a[j]->getName() == components_b[i]->getName())
                    {
                        components.push_back(components_b[i]);
                        weights.push_back(std::max(weights_a[j], weights_b[i]));
                        break;
                    }
            }

            if (a && components.size() == components_a.size())
                return a;

            if (b && components.size() == components_b.size())
                return b;

            if (components.size() == 1)
                return components[0];

            return std::make_shared<CompoundStateSpace>(components, weights);
        }
    }
}
