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

#include "ompl/base/StateManifold.h"
#include "ompl/util/Exception.h"
#include <boost/thread/mutex.hpp>
#include <boost/lexical_cast.hpp>
#include <numeric>
#include <limits>
#include <queue>
#include <cmath>
#include <list>

const std::string ompl::base::StateManifold::DEFAULT_PROJECTION_NAME = "";

/// @cond IGNORE
namespace ompl
{
    namespace base
    {
        static std::list<StateManifold*> STATE_MANIFOLD_LIST;
        static boost::mutex              STATE_MANIFOLD_LIST_LOCK;
    }
}
/// @endcond

ompl::base::StateManifold::StateManifold(void)
{
    // autocompute a unique name
    static boost::mutex lock;
    static unsigned int m = 0;

    lock.lock();
    m++;
    lock.unlock();

    name_ = "Manifold" + boost::lexical_cast<std::string>(m);
    boost::mutex::scoped_lock smLock(STATE_MANIFOLD_LIST_LOCK);
    STATE_MANIFOLD_LIST.push_back(this);

    longestValidSegment_ = 0.0;
    longestValidSegmentFraction_ = 0.01; // 1%
    longestValidSegmentCountFactor_ = 1;

    type_ = STATE_MANIFOLD_UNKNOWN;

    maxExtent_ = std::numeric_limits<double>::infinity();
}

ompl::base::StateManifold::~StateManifold(void)
{
    boost::mutex::scoped_lock smLock(STATE_MANIFOLD_LIST_LOCK);
    STATE_MANIFOLD_LIST.remove(this);
}

const std::string& ompl::base::StateManifold::getName(void) const
{
    return name_;
}

void ompl::base::StateManifold::setName(const std::string &name)
{
    name_ = name;
}

void ompl::base::StateManifold::registerProjections(void)
{
}

void ompl::base::StateManifold::setup(void)
{
    maxExtent_ = getMaximumExtent();
    longestValidSegment_ = maxExtent_ * longestValidSegmentFraction_;

    if (longestValidSegment_ < std::numeric_limits<double>::epsilon())
        throw Exception("The longest valid segment for manifold " + getName() + " must be positive");

    registerProjections();

    for (std::map<std::string, ProjectionEvaluatorPtr>::const_iterator it = projections_.begin() ; it != projections_.end() ; ++it)
        it->second->setup();
}

double* ompl::base::StateManifold::getValueAddressAtIndex(State *state, const unsigned int index) const
{
    return NULL;
}

void ompl::base::StateManifold::printState(const State *state, std::ostream &out) const
{
    out << "State instance [" << state << ']' << std::endl;
}

void ompl::base::StateManifold::printSettings(std::ostream &out) const
{
    out << "StateManifold '" << getName() << "' instance: " << this << std::endl;
    printProjections(out);
}

void ompl::base::StateManifold::printProjections(std::ostream &out) const
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
        static bool StateManifoldIncludes(const StateManifold *self, const StateManifold *other)
        {
            std::queue<const StateManifold*> q;
            q.push(self);
            while (!q.empty())
            {
                const StateManifold *m = q.front();
                q.pop();
                if (m->getName() == other->getName())
                    return true;
                if (m->isCompound())
                {
                    unsigned int c = m->as<CompoundStateManifold>()->getSubManifoldCount();
                    for (unsigned int i = 0 ; i < c ; ++i)
                        q.push(m->as<CompoundStateManifold>()->getSubManifold(i).get());
                }
            }
            return false;
        }

        static bool StateManifoldCovers(const StateManifold *self, const StateManifold *other)
        {
            if (StateManifoldIncludes(self, other))
                return true;
            else
                if (other->isCompound())
                {
                    unsigned int c = other->as<CompoundStateManifold>()->getSubManifoldCount();
                    for (unsigned int i = 0 ; i < c ; ++i)
                        if (!StateManifoldCovers(self, other->as<CompoundStateManifold>()->getSubManifold(i).get()))
                            return false;
                    return true;
                }
            return false;
        }
    }
}

/// @endcond

bool ompl::base::StateManifold::covers(const StateManifoldPtr &other) const
{
    return StateManifoldCovers(this, other.get());
}

bool ompl::base::StateManifold::includes(const StateManifoldPtr &other) const
{
    return StateManifoldIncludes(this, other.get());
}

void ompl::base::StateManifold::diagram(std::ostream &out)
{
    boost::mutex::scoped_lock smLock(STATE_MANIFOLD_LIST_LOCK);
    out << "digraph StateManifolds {" << std::endl;
    for (std::list<StateManifold*>::iterator it = STATE_MANIFOLD_LIST.begin() ; it != STATE_MANIFOLD_LIST.end(); ++it)
    {
        out << '"' << (*it)->getName() << '"' << std::endl;
        for (std::list<StateManifold*>::iterator jt = STATE_MANIFOLD_LIST.begin() ; jt != STATE_MANIFOLD_LIST.end(); ++jt)
            if (it != jt)
            {
                if ((*it)->isCompound() && (*it)->as<CompoundStateManifold>()->hasSubManifold((*jt)->getName()))
                    out << '"' << (*it)->getName() << "\" -> \"" << (*jt)->getName() << "\" [label=\"" <<
                        boost::lexical_cast<std::string>((*it)->as<CompoundStateManifold>()->getSubManifoldWeight((*jt)->getName())) <<
                        "\"];" << std::endl;
                else
                    if (!StateManifoldIncludes(*it, *jt) && StateManifoldCovers(*it, *jt))
                        out << '"' << (*it)->getName() << "\" -> \"" << (*jt)->getName() << "\" [style=dashed];" << std::endl;
            }
    }
    out << '}' << std::endl;
}

bool ompl::base::StateManifold::hasDefaultProjection(void) const
{
    return hasProjection(DEFAULT_PROJECTION_NAME);
}

bool ompl::base::StateManifold::hasProjection(const std::string &name) const
{
    return projections_.find(name) != projections_.end();
}

ompl::base::ProjectionEvaluatorPtr ompl::base::StateManifold::getDefaultProjection(void) const
{
    if (hasDefaultProjection())
        return getProjection(DEFAULT_PROJECTION_NAME);
    else
    {
        msg_.error("No default projection is set");
        return ProjectionEvaluatorPtr();
    }
}

ompl::base::ProjectionEvaluatorPtr ompl::base::StateManifold::getProjection(const std::string &name) const
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

const std::map<std::string, ompl::base::ProjectionEvaluatorPtr>& ompl::base::StateManifold::getRegisteredProjections(void) const
{
    return projections_;
}

void ompl::base::StateManifold::registerDefaultProjection(const ProjectionEvaluatorPtr &projection)
{
    registerProjection(DEFAULT_PROJECTION_NAME, projection);
}

void ompl::base::StateManifold::registerProjection(const std::string &name, const ProjectionEvaluatorPtr &projection)
{
    if (projection)
        projections_[name] = projection;
    else
        msg_.error("Attempting to register invalid projection under name '%s'. Ignoring.", name.c_str());
}

bool ompl::base::StateManifold::isCompound(void) const
{
    return false;
}

void ompl::base::StateManifold::setValidSegmentCountFactor(unsigned int factor)
{
    if (factor < 1)
        throw Exception("The multiplicative factor for the valid segment count between two states must be strictly positive");
    longestValidSegmentCountFactor_ = factor;
}

void ompl::base::StateManifold::setLongestValidSegmentFraction(double segmentFraction)
{
    if (segmentFraction < std::numeric_limits<double>::epsilon() || segmentFraction > 1.0 - std::numeric_limits<double>::epsilon())
        throw Exception("The fraction of the extent must be larger than 0 and less than 1");
    longestValidSegmentFraction_ = segmentFraction;
}

unsigned int ompl::base::StateManifold::getValidSegmentCountFactor(void) const
{
    return longestValidSegmentCountFactor_;
}

double ompl::base::StateManifold::getLongestValidSegmentFraction(void) const
{
    return longestValidSegmentFraction_;
}

unsigned int ompl::base::StateManifold::validSegmentCount(const State *state1, const State *state2) const
{
    return longestValidSegmentCountFactor_ * (unsigned int)ceil(distance(state1, state2) / longestValidSegment_);
}

ompl::base::CompoundStateManifold::CompoundStateManifold(void) : StateManifold(), componentCount_(0), locked_(false)
{
    setName("Compound" + getName());
}

ompl::base::CompoundStateManifold::CompoundStateManifold(const std::vector<StateManifoldPtr> &components,
                                                         const std::vector<double> &weights) : StateManifold(), componentCount_(0), locked_(false)
{
    if (components.size() != weights.size())
        throw Exception("Number of component manifolds and weights are not the same");
    setName("Compound" + getName());
    for (unsigned int i = 0 ; i < components.size() ; ++i)
        addSubManifold(components[i], weights[i]);
}

void ompl::base::CompoundStateManifold::addSubManifold(const StateManifoldPtr &component, double weight)
{
    if (locked_)
        throw Exception("This manifold is locked. No further components can be added");
    if (weight < 0.0)
        throw Exception("Submanifold weight cannot be negative");
    components_.push_back(component);
    weights_.push_back(weight);
    componentCount_ = components_.size();
}

bool ompl::base::CompoundStateManifold::isCompound(void) const
{
    return true;
}

unsigned int ompl::base::CompoundStateManifold::getSubManifoldCount(void) const
{
    return componentCount_;
}

const ompl::base::StateManifoldPtr& ompl::base::CompoundStateManifold::getSubManifold(const unsigned int index) const
{
    if (componentCount_ > index)
        return components_[index];
    else
        throw Exception("Submanifold index does not exist");
}

bool ompl::base::CompoundStateManifold::hasSubManifold(const std::string &name) const
{
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
        if (components_[i]->getName() == name)
            return true;
    return false;
}

unsigned int ompl::base::CompoundStateManifold::getSubManifoldIndex(const std::string& name) const
{
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
        if (components_[i]->getName() == name)
            return i;
    throw Exception("Submanifold " + name + " does not exist");
}

const ompl::base::StateManifoldPtr& ompl::base::CompoundStateManifold::getSubManifold(const std::string& name) const
{
    return components_[getSubManifoldIndex(name)];
}

double ompl::base::CompoundStateManifold::getSubManifoldWeight(const unsigned int index) const
{
    if (componentCount_ > index)
        return weights_[index];
    else
        throw Exception("Submanifold index does not exist");
}

double ompl::base::CompoundStateManifold::getSubManifoldWeight(const std::string &name) const
{
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
        if (components_[i]->getName() == name)
            return weights_[i];
    throw Exception("Submanifold " + name + " does not exist");
}

void ompl::base::CompoundStateManifold::setSubManifoldWeight(const unsigned int index, double weight)
{
    if (weight < 0.0)
        throw Exception("Submanifold weight cannot be negative");
    if (componentCount_ > index)
        weights_[index] = weight;
    else
        throw Exception("Submanifold index does not exist");
}

void ompl::base::CompoundStateManifold::setSubManifoldWeight(const std::string &name, double weight)
{
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
        if (components_[i]->getName() == name)
        {
            setSubManifoldWeight(i, weight);
            return;
        }
    throw Exception("Submanifold " + name + " does not exist");
}

const std::vector<ompl::base::StateManifoldPtr>& ompl::base::CompoundStateManifold::getSubManifolds(void) const
{
    return components_;
}

const std::vector<double>& ompl::base::CompoundStateManifold::getSubManifoldWeights(void) const
{
    return weights_;
}

unsigned int ompl::base::CompoundStateManifold::getDimension(void) const
{
    unsigned int dim = 0;
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
        dim += components_[i]->getDimension();
    return dim;
}

double ompl::base::CompoundStateManifold::getMaximumExtent(void) const
{
    double e = 0.0;
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
        e += weights_[i] * components_[i]->getMaximumExtent();
    return e;
}

void ompl::base::CompoundStateManifold::enforceBounds(State *state) const
{
    CompoundState *cstate = static_cast<CompoundState*>(state);
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
        components_[i]->enforceBounds(cstate->components[i]);
}

bool ompl::base::CompoundStateManifold::satisfiesBounds(const State *state) const
{
    const CompoundState *cstate = static_cast<const CompoundState*>(state);
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
        if (!components_[i]->satisfiesBounds(cstate->components[i]))
            return false;
    return true;
}

void ompl::base::CompoundStateManifold::copyState(State *destination, const State *source) const
{
    CompoundState      *cdest = static_cast<CompoundState*>(destination);
    const CompoundState *csrc = static_cast<const CompoundState*>(source);
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
        components_[i]->copyState(cdest->components[i], csrc->components[i]);
}

double ompl::base::CompoundStateManifold::distance(const State *state1, const State *state2) const
{
    const CompoundState *cstate1 = static_cast<const CompoundState*>(state1);
    const CompoundState *cstate2 = static_cast<const CompoundState*>(state2);
    double dist = 0.0;
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
        dist += weights_[i] * components_[i]->distance(cstate1->components[i], cstate2->components[i]);
    return dist;
}

void ompl::base::CompoundStateManifold::setLongestValidSegmentFraction(double segmentFraction)
{
    StateManifold::setLongestValidSegmentFraction(segmentFraction);
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
        components_[i]->setLongestValidSegmentFraction(segmentFraction);
}

unsigned int ompl::base::CompoundStateManifold::validSegmentCount(const State *state1, const State *state2) const
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

bool ompl::base::CompoundStateManifold::equalStates(const State *state1, const State *state2) const
{
    const CompoundState *cstate1 = static_cast<const CompoundState*>(state1);
    const CompoundState *cstate2 = static_cast<const CompoundState*>(state2);
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
        if (!components_[i]->equalStates(cstate1->components[i], cstate2->components[i]))
            return false;
    return true;
}

void ompl::base::CompoundStateManifold::interpolate(const State *from, const State *to, const double t, State *state) const
{
    const CompoundState *cfrom  = static_cast<const CompoundState*>(from);
    const CompoundState *cto    = static_cast<const CompoundState*>(to);
    CompoundState       *cstate = static_cast<CompoundState*>(state);
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
        components_[i]->interpolate(cfrom->components[i], cto->components[i], t, cstate->components[i]);
}

ompl::base::ManifoldStateSamplerPtr ompl::base::CompoundStateManifold::allocStateSampler(void) const
{
    double totalWeight = std::accumulate(weights_.begin(), weights_.end(), 0.0);
    if (totalWeight < std::numeric_limits<double>::epsilon())
        totalWeight = 1.0;
    CompoundManifoldStateSampler *ss = new CompoundManifoldStateSampler(this);
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
        ss->addSampler(components_[i]->allocStateSampler(), weights_[i] / totalWeight);
    return ManifoldStateSamplerPtr(ss);
}

ompl::base::State* ompl::base::CompoundStateManifold::allocState(void) const
{
    CompoundState *state = new CompoundState();
    allocStateComponents(state);
    return static_cast<State*>(state);
}

void ompl::base::CompoundStateManifold::allocStateComponents(CompoundState *state) const
{
    state->components = new State*[componentCount_];
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
        state->components[i] = components_[i]->allocState();
}

void ompl::base::CompoundStateManifold::freeState(State *state) const
{
    CompoundState *cstate = static_cast<CompoundState*>(state);
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
        components_[i]->freeState(cstate->components[i]);
    delete[] cstate->components;
    delete cstate;
}

void ompl::base::CompoundStateManifold::lock(void)
{
    locked_ = true;
}

bool ompl::base::CompoundStateManifold::isLocked(void) const
{
    return locked_;
}

double* ompl::base::CompoundStateManifold::getValueAddressAtIndex(State *state, const unsigned int index) const
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

void ompl::base::CompoundStateManifold::printState(const State *state, std::ostream &out) const
{
    out << "Compound state [" << std::endl;
    const CompoundState *cstate = static_cast<const CompoundState*>(state);
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
        components_[i]->printState(cstate->components[i], out);
    out << "]" << std::endl;
}

void ompl::base::CompoundStateManifold::printSettings(std::ostream &out) const
{
    out << "Compound state manifold '" << getName() << "' of dimension " << getDimension() << (isLocked() ? " (locked)" : "") << " [" << std::endl;
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
    {
        components_[i]->printSettings(out);
        out << " of weight " << weights_[i] << std::endl;
    }
    out << "]" << std::endl;
    printProjections(out);
}

void ompl::base::CompoundStateManifold::setup(void)
{
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
        components_[i]->setup();

    StateManifold::setup();
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
        int copyStateData(const StateManifoldPtr &destM, State *dest, const StateManifoldPtr &sourceM, const State *source)
        {
            // if states correspond to the same manifold, simply do copy
            if (destM->getName() == sourceM->getName())
            {
                if (dest != source)
                    destM->copyState(dest, source);
                return ALL_DATA_COPIED;
            }

            int result = NO_DATA_COPIED;

            // if "to" state is compound
            if (destM->isCompound())
            {
                const CompoundStateManifold *compoundDestM = destM->as<CompoundStateManifold>();
                CompoundState *compoundDest = dest->as<CompoundState>();

                // if there is a submanifold in "to" that corresponds to "from", set the data and return
                for (unsigned int i = 0 ; i < compoundDestM->getSubManifoldCount() ; ++i)
                    if (compoundDestM->getSubManifold(i)->getName() == sourceM->getName())
                    {
                        if (compoundDest->components[i] != source)
                            compoundDestM->getSubManifold(i)->copyState(compoundDest->components[i], source);
                        return ALL_DATA_COPIED;
                    }

                // it could be there are further levels of compound manifolds where the data can be set
                // so we call this function recursively
                for (unsigned int i = 0 ; i < compoundDestM->getSubManifoldCount() ; ++i)
                {
                    int res = copyStateData(compoundDestM->getSubManifold(i), compoundDest->components[i], sourceM, source);

                    if (res != NO_DATA_COPIED)
                        result = SOME_DATA_COPIED;

                    // if all data was copied, we stop
                    if (res == ALL_DATA_COPIED)
                        return ALL_DATA_COPIED;
                }
            }

            // if we got to this point, it means that the data in "from" could not be copied as a chunk to "to"
            // it could be the case "from" is from a compound manifold as well, so we can copy parts of "from", as needed
            if (sourceM->isCompound())
            {
                const CompoundStateManifold *compoundSourceM = sourceM->as<CompoundStateManifold>();
                const CompoundState *compoundSource = source->as<CompoundState>();

                unsigned int copiedComponents = 0;

                // if there is a submanifold in "to" that corresponds to "from", set the data and return
                for (unsigned int i = 0 ; i < compoundSourceM->getSubManifoldCount() ; ++i)
                {
                    int res = copyStateData(destM, dest, compoundSourceM->getSubManifold(i), compoundSource->components[i]);
                    if (res == ALL_DATA_COPIED)
                        copiedComponents++;
                    if (res)
                        result = SOME_DATA_COPIED;
                }

                // if each individual component got copied, then the entire data in "from" got copied
                if (copiedComponents == compoundSourceM->getSubManifoldCount())
                    result = ALL_DATA_COPIED;
            }

            return result;
        }

        /// @cond IGNORE
        inline bool StateManifoldHasContent(const StateManifoldPtr &m)
        {
            if (!m)
                return false;
            if (m->getDimension() == 0 && m->getType() == STATE_MANIFOLD_UNKNOWN && m->isCompound())
            {
                const unsigned int nc = m->as<CompoundStateManifold>()->getSubManifoldCount();
                for (unsigned int i = 0 ; i < nc ; ++i)
                    if (StateManifoldHasContent(m->as<CompoundStateManifold>()->getSubManifold(i)))
                        return true;
                return false;
            }
            return true;
        }
        /// @endcond

        StateManifoldPtr operator+(const StateManifoldPtr &a, const StateManifoldPtr &b)
        {
            if (!StateManifoldHasContent(a) && StateManifoldHasContent(b))
                return b;

            if (!StateManifoldHasContent(b) && StateManifoldHasContent(a))
                return a;

            std::vector<StateManifoldPtr> components;
            std::vector<double>           weights;

            bool change = false;
            if (a)
            {
                bool used = false;
                if (CompoundStateManifold *csm_a = dynamic_cast<CompoundStateManifold*>(a.get()))
                    if (!csm_a->isLocked())
                    {
                        used = true;
                        for (unsigned int i = 0 ; i < csm_a->getSubManifoldCount() ; ++i)
                        {
                            components.push_back(csm_a->getSubManifold(i));
                            weights.push_back(csm_a->getSubManifoldWeight(i));
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

                if (CompoundStateManifold *csm_b = dynamic_cast<CompoundStateManifold*>(b.get()))
                    if (!csm_b->isLocked())
                    {
                        used = true;
                        for (unsigned int i = 0 ; i < csm_b->getSubManifoldCount() ; ++i)
                        {
                            bool ok = true;
                            for (unsigned int j = 0 ; j < size ; ++j)
                                if (components[j]->getName() == csm_b->getSubManifold(i)->getName())
                                {
                                    ok = false;
                                    break;
                                }
                            if (ok)
                            {
                                components.push_back(csm_b->getSubManifold(i));
                                weights.push_back(csm_b->getSubManifoldWeight(i));
                                change = true;
                            }
                        }
                        if (components.size() == csm_b->getSubManifoldCount())
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

            return StateManifoldPtr(new CompoundStateManifold(components, weights));
        }

        StateManifoldPtr operator-(const StateManifoldPtr &a, const StateManifoldPtr &b)
        {
            std::vector<StateManifoldPtr> components_a;
            std::vector<double>           weights_a;
            std::vector<StateManifoldPtr> components_b;

            if (a)
            {
                bool used = false;
                if (CompoundStateManifold *csm_a = dynamic_cast<CompoundStateManifold*>(a.get()))
                    if (!csm_a->isLocked())
                    {
                        used = true;
                        for (unsigned int i = 0 ; i < csm_a->getSubManifoldCount() ; ++i)
                        {
                            components_a.push_back(csm_a->getSubManifold(i));
                            weights_a.push_back(csm_a->getSubManifoldWeight(i));
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
                if (CompoundStateManifold *csm_b = dynamic_cast<CompoundStateManifold*>(b.get()))
                    if (!csm_b->isLocked())
                    {
                        used = true;
                        for (unsigned int i = 0 ; i < csm_b->getSubManifoldCount() ; ++i)
                            components_b.push_back(csm_b->getSubManifold(i));
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

            return StateManifoldPtr(new CompoundStateManifold(components_a, weights_a));
        }

        StateManifoldPtr operator-(const StateManifoldPtr &a, const std::string &name)
        {
            std::vector<StateManifoldPtr> components;
            std::vector<double>           weights;

            bool change = false;
            if (a)
            {
                bool used = false;
                if (CompoundStateManifold *csm_a = dynamic_cast<CompoundStateManifold*>(a.get()))
                    if (!csm_a->isLocked())
                    {
                        used = true;
                        for (unsigned int i = 0 ; i < csm_a->getSubManifoldCount() ; ++i)
                        {
                            if (csm_a->getSubManifold(i)->getName() == name)
                            {
                                change = true;
                                continue;
                            }
                            components.push_back(csm_a->getSubManifold(i));
                            weights.push_back(csm_a->getSubManifoldWeight(i));
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

            return StateManifoldPtr(new CompoundStateManifold(components, weights));
        }

        StateManifoldPtr operator*(const StateManifoldPtr &a, const StateManifoldPtr &b)
        {
            std::vector<StateManifoldPtr> components_a;
            std::vector<double>           weights_a;
            std::vector<StateManifoldPtr> components_b;
            std::vector<double>           weights_b;

            if (a)
            {
                bool used = false;
                if (CompoundStateManifold *csm_a = dynamic_cast<CompoundStateManifold*>(a.get()))
                    if (!csm_a->isLocked())
                    {
                        used = true;
                        for (unsigned int i = 0 ; i < csm_a->getSubManifoldCount() ; ++i)
                        {
                            components_a.push_back(csm_a->getSubManifold(i));
                            weights_a.push_back(csm_a->getSubManifoldWeight(i));
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
                if (CompoundStateManifold *csm_b = dynamic_cast<CompoundStateManifold*>(b.get()))
                    if (!csm_b->isLocked())
                    {
                        used = true;
                        for (unsigned int i = 0 ; i < csm_b->getSubManifoldCount() ; ++i)
                        {
                            components_b.push_back(csm_b->getSubManifold(i));
                            weights_b.push_back(csm_b->getSubManifoldWeight(i));
                        }
                    }

                if (!used)
                {
                    components_b.push_back(b);
                    weights_b.push_back(1.0);
                }
            }

            std::vector<StateManifoldPtr> components;
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

            return StateManifoldPtr(new CompoundStateManifold(components, weights));
        }

    }
}
