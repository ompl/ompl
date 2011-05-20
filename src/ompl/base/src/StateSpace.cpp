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
#include <boost/thread/mutex.hpp>
#include <boost/lexical_cast.hpp>
#include <numeric>
#include <limits>
#include <queue>
#include <cmath>
#include <list>

const std::string ompl::base::StateSpace::DEFAULT_PROJECTION_NAME = "";

/// @cond IGNORE
namespace ompl
{
    namespace base
    {
        static std::list<StateSpace*> STATE_SPACE_LIST;
        static boost::mutex           STATE_SPACE_LIST_LOCK;
    }
}
/// @endcond

ompl::base::StateSpace::StateSpace(void)
{
    // autocompute a unique name
    static boost::mutex lock;
    static unsigned int m = 0;

    lock.lock();
    m++;
    lock.unlock();

    name_ = "Space" + boost::lexical_cast<std::string>(m);
    boost::mutex::scoped_lock smLock(STATE_SPACE_LIST_LOCK);
    STATE_SPACE_LIST.push_back(this);

    longestValidSegment_ = 0.0;
    longestValidSegmentFraction_ = 0.01; // 1%
    longestValidSegmentCountFactor_ = 1;

    type_ = STATE_SPACE_UNKNOWN;

    maxExtent_ = std::numeric_limits<double>::infinity();
}

ompl::base::StateSpace::~StateSpace(void)
{
    boost::mutex::scoped_lock smLock(STATE_SPACE_LIST_LOCK);
    STATE_SPACE_LIST.remove(this);
}

const std::string& ompl::base::StateSpace::getName(void) const
{
    return name_;
}

void ompl::base::StateSpace::setName(const std::string &name)
{
    name_ = name;
}

void ompl::base::StateSpace::registerProjections(void)
{
}

void ompl::base::StateSpace::setup(void)
{
    maxExtent_ = getMaximumExtent();
    longestValidSegment_ = maxExtent_ * longestValidSegmentFraction_;

    if (longestValidSegment_ < std::numeric_limits<double>::epsilon())
        throw Exception("The longest valid segment for state space " + getName() + " must be positive");

    // make sure we don't overwrite projections that have been configured by the user
    std::map<std::string, ProjectionEvaluatorPtr> oldProjections = projections_;
    registerProjections();
    for (std::map<std::string, ProjectionEvaluatorPtr>::iterator it = oldProjections.begin() ; it != oldProjections.end() ; ++it)
        if (it->second->userConfigured())
        {
            std::map<std::string, ProjectionEvaluatorPtr>::iterator o = projections_.find(it->first);
            if (o != projections_.end())
                if (!o->second->userConfigured())
                    projections_[it->first] = it->second;
        }

    for (std::map<std::string, ProjectionEvaluatorPtr>::const_iterator it = projections_.begin() ; it != projections_.end() ; ++it)
        it->second->setup();
}

double* ompl::base::StateSpace::getValueAddressAtIndex(State *state, const unsigned int index) const
{
    return NULL;
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
        for (std::map<std::string, ProjectionEvaluatorPtr>::const_iterator it = projections_.begin() ; it != projections_.end() ; ++it)
        {
            out << "  - ";
            if (it->first == DEFAULT_PROJECTION_NAME)
                out << "<default>";
            else
                out << it->first;
            out << std::endl;
            it->second->printSettings(out);
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
            std::queue<const StateSpace*> q;
            q.push(self);
            while (!q.empty())
            {
                const StateSpace *m = q.front();
                q.pop();
                if (m->getName() == other->getName())
                    return true;
                if (m->isCompound())
                {
                    unsigned int c = m->as<CompoundStateSpace>()->getSubSpaceCount();
                    for (unsigned int i = 0 ; i < c ; ++i)
                        q.push(m->as<CompoundStateSpace>()->getSubSpace(i).get());
                }
            }
            return false;
        }

        static bool StateSpaceCovers(const StateSpace *self, const StateSpace *other)
        {
            if (StateSpaceIncludes(self, other))
                return true;
            else
                if (other->isCompound())
                {
                    unsigned int c = other->as<CompoundStateSpace>()->getSubSpaceCount();
                    for (unsigned int i = 0 ; i < c ; ++i)
                        if (!StateSpaceCovers(self, other->as<CompoundStateSpace>()->getSubSpace(i).get()))
                            return false;
                    return true;
                }
            return false;
        }
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

void ompl::base::StateSpace::diagram(std::ostream &out)
{
    boost::mutex::scoped_lock smLock(STATE_SPACE_LIST_LOCK);
    out << "digraph StateSpaces {" << std::endl;
    for (std::list<StateSpace*>::iterator it = STATE_SPACE_LIST.begin() ; it != STATE_SPACE_LIST.end(); ++it)
    {
        out << '"' << (*it)->getName() << '"' << std::endl;
        for (std::list<StateSpace*>::iterator jt = STATE_SPACE_LIST.begin() ; jt != STATE_SPACE_LIST.end(); ++jt)
            if (it != jt)
            {
                if ((*it)->isCompound() && (*it)->as<CompoundStateSpace>()->hasSubSpace((*jt)->getName()))
                    out << '"' << (*it)->getName() << "\" -> \"" << (*jt)->getName() << "\" [label=\"" <<
                        boost::lexical_cast<std::string>((*it)->as<CompoundStateSpace>()->getSubSpaceWeight((*jt)->getName())) <<
                        "\"];" << std::endl;
                else
                    if (!StateSpaceIncludes(*it, *jt) && StateSpaceCovers(*it, *jt))
                        out << '"' << (*it)->getName() << "\" -> \"" << (*jt)->getName() << "\" [style=dashed];" << std::endl;
            }
    }
    out << '}' << std::endl;
}

bool ompl::base::StateSpace::hasDefaultProjection(void) const
{
    return hasProjection(DEFAULT_PROJECTION_NAME);
}

bool ompl::base::StateSpace::hasProjection(const std::string &name) const
{
    return projections_.find(name) != projections_.end();
}

ompl::base::ProjectionEvaluatorPtr ompl::base::StateSpace::getDefaultProjection(void) const
{
    if (hasDefaultProjection())
        return getProjection(DEFAULT_PROJECTION_NAME);
    else
    {
        msg_.error("No default projection is set");
        return ProjectionEvaluatorPtr();
    }
}

ompl::base::ProjectionEvaluatorPtr ompl::base::StateSpace::getProjection(const std::string &name) const
{
    std::map<std::string, ProjectionEvaluatorPtr>::const_iterator it = projections_.find(name);
    if (it != projections_.end())
        return it->second;
    else
    {
        msg_.error("Projection '" + name + "' is not defined");
        return ProjectionEvaluatorPtr();
    }
}

const std::map<std::string, ompl::base::ProjectionEvaluatorPtr>& ompl::base::StateSpace::getRegisteredProjections(void) const
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
        msg_.error("Attempting to register invalid projection under name '%s'. Ignoring.", name.c_str());
}

bool ompl::base::StateSpace::isCompound(void) const
{
    return false;
}

void ompl::base::StateSpace::setValidSegmentCountFactor(unsigned int factor)
{
    if (factor < 1)
        throw Exception("The multiplicative factor for the valid segment count between two states must be strictly positive");
    longestValidSegmentCountFactor_ = factor;
}

void ompl::base::StateSpace::setLongestValidSegmentFraction(double segmentFraction)
{
    if (segmentFraction < std::numeric_limits<double>::epsilon() || segmentFraction > 1.0 - std::numeric_limits<double>::epsilon())
        throw Exception("The fraction of the extent must be larger than 0 and less than 1");
    longestValidSegmentFraction_ = segmentFraction;
}

unsigned int ompl::base::StateSpace::getValidSegmentCountFactor(void) const
{
    return longestValidSegmentCountFactor_;
}

double ompl::base::StateSpace::getLongestValidSegmentFraction(void) const
{
    return longestValidSegmentFraction_;
}

unsigned int ompl::base::StateSpace::validSegmentCount(const State *state1, const State *state2) const
{
    return longestValidSegmentCountFactor_ * (unsigned int)ceil(distance(state1, state2) / longestValidSegment_);
}

ompl::base::CompoundStateSpace::CompoundStateSpace(void) : StateSpace(), componentCount_(0), locked_(false)
{
    setName("Compound" + getName());
}

ompl::base::CompoundStateSpace::CompoundStateSpace(const std::vector<StateSpacePtr> &components,
                                                         const std::vector<double> &weights) : StateSpace(), componentCount_(0), locked_(false)
{
    if (components.size() != weights.size())
        throw Exception("Number of component spaces and weights are not the same");
    setName("Compound" + getName());
    for (unsigned int i = 0 ; i < components.size() ; ++i)
        addSubSpace(components[i], weights[i]);
}

void ompl::base::CompoundStateSpace::addSubSpace(const StateSpacePtr &component, double weight)
{
    if (locked_)
        throw Exception("This state space is locked. No further components can be added");
    if (weight < 0.0)
        throw Exception("Subspace weight cannot be negative");
    components_.push_back(component);
    weights_.push_back(weight);
    componentCount_ = components_.size();
}

bool ompl::base::CompoundStateSpace::isCompound(void) const
{
    return true;
}

unsigned int ompl::base::CompoundStateSpace::getSubSpaceCount(void) const
{
    return componentCount_;
}

const ompl::base::StateSpacePtr& ompl::base::CompoundStateSpace::getSubSpace(const unsigned int index) const
{
    if (componentCount_ > index)
        return components_[index];
    else
        throw Exception("Subspace index does not exist");
}

bool ompl::base::CompoundStateSpace::hasSubSpace(const std::string &name) const
{
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
        if (components_[i]->getName() == name)
            return true;
    return false;
}

unsigned int ompl::base::CompoundStateSpace::getSubSpaceIndex(const std::string& name) const
{
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
        if (components_[i]->getName() == name)
            return i;
    throw Exception("Subspace " + name + " does not exist");
}

const ompl::base::StateSpacePtr& ompl::base::CompoundStateSpace::getSubSpace(const std::string& name) const
{
    return components_[getSubSpaceIndex(name)];
}

double ompl::base::CompoundStateSpace::getSubSpaceWeight(const unsigned int index) const
{
    if (componentCount_ > index)
        return weights_[index];
    else
        throw Exception("Subspace index does not exist");
}

double ompl::base::CompoundStateSpace::getSubSpaceWeight(const std::string &name) const
{
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
        if (components_[i]->getName() == name)
            return weights_[i];
    throw Exception("Subspace " + name + " does not exist");
}

void ompl::base::CompoundStateSpace::setSubSpaceWeight(const unsigned int index, double weight)
{
    if (weight < 0.0)
        throw Exception("Subspace weight cannot be negative");
    if (componentCount_ > index)
        weights_[index] = weight;
    else
        throw Exception("Subspace index does not exist");
}

void ompl::base::CompoundStateSpace::setSubSpaceWeight(const std::string &name, double weight)
{
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
        if (components_[i]->getName() == name)
        {
            setSubSpaceWeight(i, weight);
            return;
        }
    throw Exception("Subspace " + name + " does not exist");
}

const std::vector<ompl::base::StateSpacePtr>& ompl::base::CompoundStateSpace::getSubSpaces(void) const
{
    return components_;
}

const std::vector<double>& ompl::base::CompoundStateSpace::getSubSpaceWeights(void) const
{
    return weights_;
}

unsigned int ompl::base::CompoundStateSpace::getDimension(void) const
{
    unsigned int dim = 0;
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
        dim += components_[i]->getDimension();
    return dim;
}

double ompl::base::CompoundStateSpace::getMaximumExtent(void) const
{
    double e = 0.0;
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
        e += weights_[i] * components_[i]->getMaximumExtent();
    return e;
}

void ompl::base::CompoundStateSpace::enforceBounds(State *state) const
{
    CompoundState *cstate = static_cast<CompoundState*>(state);
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
        components_[i]->enforceBounds(cstate->components[i]);
}

bool ompl::base::CompoundStateSpace::satisfiesBounds(const State *state) const
{
    const CompoundState *cstate = static_cast<const CompoundState*>(state);
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
        if (!components_[i]->satisfiesBounds(cstate->components[i]))
            return false;
    return true;
}

void ompl::base::CompoundStateSpace::copyState(State *destination, const State *source) const
{
    CompoundState      *cdest = static_cast<CompoundState*>(destination);
    const CompoundState *csrc = static_cast<const CompoundState*>(source);
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
        components_[i]->copyState(cdest->components[i], csrc->components[i]);
}

double ompl::base::CompoundStateSpace::distance(const State *state1, const State *state2) const
{
    const CompoundState *cstate1 = static_cast<const CompoundState*>(state1);
    const CompoundState *cstate2 = static_cast<const CompoundState*>(state2);
    double dist = 0.0;
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
        dist += weights_[i] * components_[i]->distance(cstate1->components[i], cstate2->components[i]);
    return dist;
}

void ompl::base::CompoundStateSpace::setLongestValidSegmentFraction(double segmentFraction)
{
    StateSpace::setLongestValidSegmentFraction(segmentFraction);
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
        components_[i]->setLongestValidSegmentFraction(segmentFraction);
}

unsigned int ompl::base::CompoundStateSpace::validSegmentCount(const State *state1, const State *state2) const
{
    const CompoundState *cstate1 = static_cast<const CompoundState*>(state1);
    const CompoundState *cstate2 = static_cast<const CompoundState*>(state2);
    unsigned int sc = 0;
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
    {
        unsigned int sci = components_[i]->validSegmentCount(cstate1->components[i], cstate2->components[i]);
        if (sci > sc)
            sc = sci;
    }
    return sc;
}

bool ompl::base::CompoundStateSpace::equalStates(const State *state1, const State *state2) const
{
    const CompoundState *cstate1 = static_cast<const CompoundState*>(state1);
    const CompoundState *cstate2 = static_cast<const CompoundState*>(state2);
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
        if (!components_[i]->equalStates(cstate1->components[i], cstate2->components[i]))
            return false;
    return true;
}

void ompl::base::CompoundStateSpace::interpolate(const State *from, const State *to, const double t, State *state) const
{
    const CompoundState *cfrom  = static_cast<const CompoundState*>(from);
    const CompoundState *cto    = static_cast<const CompoundState*>(to);
    CompoundState       *cstate = static_cast<CompoundState*>(state);
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
        components_[i]->interpolate(cfrom->components[i], cto->components[i], t, cstate->components[i]);
}

ompl::base::StateSamplerPtr ompl::base::CompoundStateSpace::allocStateSampler(void) const
{
    double totalWeight = std::accumulate(weights_.begin(), weights_.end(), 0.0);
    if (totalWeight < std::numeric_limits<double>::epsilon())
        totalWeight = 1.0;
    CompoundStateSampler *ss = new CompoundStateSampler(this);
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
        ss->addSampler(components_[i]->allocStateSampler(), weights_[i] / totalWeight);
    return StateSamplerPtr(ss);
}

ompl::base::State* ompl::base::CompoundStateSpace::allocState(void) const
{
    CompoundState *state = new CompoundState();
    allocStateComponents(state);
    return static_cast<State*>(state);
}

void ompl::base::CompoundStateSpace::allocStateComponents(CompoundState *state) const
{
    state->components = new State*[componentCount_];
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
        state->components[i] = components_[i]->allocState();
}

void ompl::base::CompoundStateSpace::freeState(State *state) const
{
    CompoundState *cstate = static_cast<CompoundState*>(state);
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
        components_[i]->freeState(cstate->components[i]);
    delete[] cstate->components;
    delete cstate;
}

void ompl::base::CompoundStateSpace::lock(void)
{
    locked_ = true;
}

bool ompl::base::CompoundStateSpace::isLocked(void) const
{
    return locked_;
}

double* ompl::base::CompoundStateSpace::getValueAddressAtIndex(State *state, const unsigned int index) const
{
    CompoundState *cstate = static_cast<CompoundState*>(state);
    unsigned int idx = 0;

    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
        for (unsigned int j = 0 ; j <= index ; ++j)
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
    return NULL;
}

void ompl::base::CompoundStateSpace::printState(const State *state, std::ostream &out) const
{
    out << "Compound state [" << std::endl;
    const CompoundState *cstate = static_cast<const CompoundState*>(state);
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
        components_[i]->printState(cstate->components[i], out);
    out << "]" << std::endl;
}

void ompl::base::CompoundStateSpace::printSettings(std::ostream &out) const
{
    out << "Compound state space '" << getName() << "' of dimension " << getDimension() << (isLocked() ? " (locked)" : "") << " [" << std::endl;
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
    {
        components_[i]->printSettings(out);
        out << " of weight " << weights_[i] << std::endl;
    }
    out << "]" << std::endl;
    printProjections(out);
}

void ompl::base::CompoundStateSpace::setup(void)
{
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
        components_[i]->setup();

    StateSpace::setup();
}

namespace ompl
{
    namespace base
    {

        /// @cond IGNORE

        static const int NO_DATA_COPIED   = 0;
        static const int SOME_DATA_COPIED = 1;
        static const int ALL_DATA_COPIED  = 2;

        /// @endcond

        // return one of the constants defined above
        int copyStateData(const StateSpacePtr &destS, State *dest, const StateSpacePtr &sourceS, const State *source)
        {
            // if states correspond to the same space, simply do copy
            if (destS->getName() == sourceS->getName())
            {
                if (dest != source)
                    destS->copyState(dest, source);
                return ALL_DATA_COPIED;
            }

            int result = NO_DATA_COPIED;

            // if "to" state is compound
            if (destS->isCompound())
            {
                const CompoundStateSpace *compoundDestS = destS->as<CompoundStateSpace>();
                CompoundState *compoundDest = dest->as<CompoundState>();

                // if there is a subspace in "to" that corresponds to "from", set the data and return
                for (unsigned int i = 0 ; i < compoundDestS->getSubSpaceCount() ; ++i)
                    if (compoundDestS->getSubSpace(i)->getName() == sourceS->getName())
                    {
                        if (compoundDest->components[i] != source)
                            compoundDestS->getSubSpace(i)->copyState(compoundDest->components[i], source);
                        return ALL_DATA_COPIED;
                    }

                // it could be there are further levels of compound spaces where the data can be set
                // so we call this function recursively
                for (unsigned int i = 0 ; i < compoundDestS->getSubSpaceCount() ; ++i)
                {
                    int res = copyStateData(compoundDestS->getSubSpace(i), compoundDest->components[i], sourceS, source);

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
                const CompoundStateSpace *compoundSourceS = sourceS->as<CompoundStateSpace>();
                const CompoundState *compoundSource = source->as<CompoundState>();

                unsigned int copiedComponents = 0;

                // if there is a subspace in "to" that corresponds to "from", set the data and return
                for (unsigned int i = 0 ; i < compoundSourceS->getSubSpaceCount() ; ++i)
                {
                    int res = copyStateData(destS, dest, compoundSourceS->getSubSpace(i), compoundSource->components[i]);
                    if (res == ALL_DATA_COPIED)
                        copiedComponents++;
                    if (res)
                        result = SOME_DATA_COPIED;
                }

                // if each individual component got copied, then the entire data in "from" got copied
                if (copiedComponents == compoundSourceS->getSubSpaceCount())
                    result = ALL_DATA_COPIED;
            }

            return result;
        }

        /// @cond IGNORE
        inline bool StateSpaceHasContent(const StateSpacePtr &m)
        {
            if (!m)
                return false;
            if (m->getDimension() == 0 && m->getType() == STATE_SPACE_UNKNOWN && m->isCompound())
            {
                const unsigned int nc = m->as<CompoundStateSpace>()->getSubSpaceCount();
                for (unsigned int i = 0 ; i < nc ; ++i)
                    if (StateSpaceHasContent(m->as<CompoundStateSpace>()->getSubSpace(i)))
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
            std::vector<double>           weights;

            bool change = false;
            if (a)
            {
                bool used = false;
                if (CompoundStateSpace *csm_a = dynamic_cast<CompoundStateSpace*>(a.get()))
                    if (!csm_a->isLocked())
                    {
                        used = true;
                        for (unsigned int i = 0 ; i < csm_a->getSubSpaceCount() ; ++i)
                        {
                            components.push_back(csm_a->getSubSpace(i));
                            weights.push_back(csm_a->getSubSpaceWeight(i));
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

                if (CompoundStateSpace *csm_b = dynamic_cast<CompoundStateSpace*>(b.get()))
                    if (!csm_b->isLocked())
                    {
                        used = true;
                        for (unsigned int i = 0 ; i < csm_b->getSubSpaceCount() ; ++i)
                        {
                            bool ok = true;
                            for (unsigned int j = 0 ; j < size ; ++j)
                                if (components[j]->getName() == csm_b->getSubSpace(i)->getName())
                                {
                                    ok = false;
                                    break;
                                }
                            if (ok)
                            {
                                components.push_back(csm_b->getSubSpace(i));
                                weights.push_back(csm_b->getSubSpaceWeight(i));
                                change = true;
                            }
                        }
                        if (components.size() == csm_b->getSubSpaceCount())
                            return b;
                    }

                if (!used)
                {
                    bool ok = true;
                    for (unsigned int j = 0 ; j < size ; ++j)
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

            return StateSpacePtr(new CompoundStateSpace(components, weights));
        }

        StateSpacePtr operator-(const StateSpacePtr &a, const StateSpacePtr &b)
        {
            std::vector<StateSpacePtr> components_a;
            std::vector<double>           weights_a;
            std::vector<StateSpacePtr> components_b;

            if (a)
            {
                bool used = false;
                if (CompoundStateSpace *csm_a = dynamic_cast<CompoundStateSpace*>(a.get()))
                    if (!csm_a->isLocked())
                    {
                        used = true;
                        for (unsigned int i = 0 ; i < csm_a->getSubSpaceCount() ; ++i)
                        {
                            components_a.push_back(csm_a->getSubSpace(i));
                            weights_a.push_back(csm_a->getSubSpaceWeight(i));
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
                if (CompoundStateSpace *csm_b = dynamic_cast<CompoundStateSpace*>(b.get()))
                    if (!csm_b->isLocked())
                    {
                        used = true;
                        for (unsigned int i = 0 ; i < csm_b->getSubSpaceCount() ; ++i)
                            components_b.push_back(csm_b->getSubSpace(i));
                    }
                if (!used)
                    components_b.push_back(b);
            }

            bool change = false;
            for (unsigned int i = 0 ; i < components_b.size() ; ++i)
                for (unsigned int j = 0 ; j < components_a.size() ; ++j)
                    if (components_a[j]->getName() == components_b[i]->getName())
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

            return StateSpacePtr(new CompoundStateSpace(components_a, weights_a));
        }

        StateSpacePtr operator-(const StateSpacePtr &a, const std::string &name)
        {
            std::vector<StateSpacePtr> components;
            std::vector<double>           weights;

            bool change = false;
            if (a)
            {
                bool used = false;
                if (CompoundStateSpace *csm_a = dynamic_cast<CompoundStateSpace*>(a.get()))
                    if (!csm_a->isLocked())
                    {
                        used = true;
                        for (unsigned int i = 0 ; i < csm_a->getSubSpaceCount() ; ++i)
                        {
                            if (csm_a->getSubSpace(i)->getName() == name)
                            {
                                change = true;
                                continue;
                            }
                            components.push_back(csm_a->getSubSpace(i));
                            weights.push_back(csm_a->getSubSpaceWeight(i));
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

            return StateSpacePtr(new CompoundStateSpace(components, weights));
        }

        StateSpacePtr operator*(const StateSpacePtr &a, const StateSpacePtr &b)
        {
            std::vector<StateSpacePtr> components_a;
            std::vector<double>           weights_a;
            std::vector<StateSpacePtr> components_b;
            std::vector<double>           weights_b;

            if (a)
            {
                bool used = false;
                if (CompoundStateSpace *csm_a = dynamic_cast<CompoundStateSpace*>(a.get()))
                    if (!csm_a->isLocked())
                    {
                        used = true;
                        for (unsigned int i = 0 ; i < csm_a->getSubSpaceCount() ; ++i)
                        {
                            components_a.push_back(csm_a->getSubSpace(i));
                            weights_a.push_back(csm_a->getSubSpaceWeight(i));
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
                if (CompoundStateSpace *csm_b = dynamic_cast<CompoundStateSpace*>(b.get()))
                    if (!csm_b->isLocked())
                    {
                        used = true;
                        for (unsigned int i = 0 ; i < csm_b->getSubSpaceCount() ; ++i)
                        {
                            components_b.push_back(csm_b->getSubSpace(i));
                            weights_b.push_back(csm_b->getSubSpaceWeight(i));
                        }
                    }

                if (!used)
                {
                    components_b.push_back(b);
                    weights_b.push_back(1.0);
                }
            }

            std::vector<StateSpacePtr> components;
            std::vector<double>           weights;

            for (unsigned int i = 0 ; i < components_b.size() ; ++i)
            {
                for (unsigned int j = 0 ; j < components_a.size() ; ++j)
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

            return StateSpacePtr(new CompoundStateSpace(components, weights));
        }

    }
}
