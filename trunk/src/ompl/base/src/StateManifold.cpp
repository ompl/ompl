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
#include <sstream>
#include <numeric>
#include <limits>

const std::string ompl::base::StateManifold::DEFAULT_PROJECTION_NAME = "";

ompl::base::StateManifold::StateManifold(void)
{
    // autocompute a unique name
    static boost::mutex lock;
    static unsigned int m = 0;
    
    lock.lock();
    m++;
    lock.unlock();

    std::stringstream ss;
    ss << "manifold" << m;
    name_ = ss.str();

    maxExtent_ = std::numeric_limits<double>::infinity();
}

void ompl::base::StateManifold::setup(void)
{
    maxExtent_ = getMaximumExtent();
}

void ompl::base::StateManifold::printState(const State *state, std::ostream &out) const
{
    out << "State instance [" << state << ']' << std::endl;
}

void ompl::base::StateManifold::printSettings(std::ostream &out) const
{
    out << "StateManifold '" << name_ << "' instance: " << this << std::endl;
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

bool ompl::base::StateManifold::haveDefaultProjection(void) const
{
    return haveProjection(DEFAULT_PROJECTION_NAME);
}

bool ompl::base::StateManifold::haveProjection(const std::string &name) const
{
    return projections_.find(name) != projections_.end();
}

ompl::base::ProjectionEvaluatorPtr ompl::base::StateManifold::getDefaultProjection(void) const
{
    if (haveDefaultProjection())
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

void ompl::base::StateManifold::registerProjection(const std::string &name, const ProjectionEvaluatorPtr &projection)
{
    if (projection)
	projections_[name] = projection;
    else
	msg_.error("Attempting to register invalid projection under name '%s'. Ignoring.", name.c_str());
}

double ompl::base::StateManifold::distanceAsFraction(const State *state1, const State *state2) const
{
    return distance(state1, state2) / maxExtent_;
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

const ompl::base::StateManifoldPtr& ompl::base::CompoundStateManifold::getSubManifold(const std::string& name) const
{
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
	if (components_[i]->getName() == name)
	    return components_[i];
    throw Exception("Submanifold " + name + " does not exist");
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

double ompl::base::CompoundStateManifold::distanceAsFraction(const State *state1, const State *state2) const
{
    const CompoundState *cstate1 = static_cast<const CompoundState*>(state1);
    const CompoundState *cstate2 = static_cast<const CompoundState*>(state2);
    double dist = 0.0;
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
    {
	double d = components_[i]->distanceAsFraction(cstate1->components[i], cstate2->components[i]);
	if (d > dist)
	    dist = d;
    }
    return dist;
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
    out << "Compound state manifold '" << name_ << "' [" << std::endl;
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
	components_[i]->printSettings(out);
    out << "]" << std::endl;
    printProjections(out);
}
	
void ompl::base::CompoundStateManifold::setup(void)
{
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
	components_[i]->setup();
    StateManifold::setup();
}
