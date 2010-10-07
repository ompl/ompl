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

#include "ompl/base/manifolds/RealVectorStateManifold.h"
#include "ompl/base/manifolds/RealVectorStateProjections.h"
#include "ompl/util/Exception.h"
#include <algorithm>
#include <cstring>
#include <limits>
#include <cmath>

void ompl::base::RealVectorStateSampler::sampleUniform(State *state)
{
    const unsigned int dim = manifold_->getDimension();
    const RealVectorBounds &bounds = static_cast<const RealVectorStateManifold*>(manifold_)->getBounds();
    
    RealVectorStateManifold::StateType *rstate = static_cast<RealVectorStateManifold::StateType*>(state);
    for (unsigned int i = 0 ; i < dim ; ++i)
	rstate->values[i] = rng_.uniformReal(bounds.low[i], bounds.high[i]);
}

void ompl::base::RealVectorStateSampler::sampleUniformNear(State *state, const State *near, const double distance)
{
    const unsigned int dim = manifold_->getDimension();
    const RealVectorBounds &bounds = static_cast<const RealVectorStateManifold*>(manifold_)->getBounds();

    RealVectorStateManifold::StateType *rstate = static_cast<RealVectorStateManifold::StateType*>(state);
    const RealVectorStateManifold::StateType *rnear = static_cast<const RealVectorStateManifold::StateType*>(near);
    for (unsigned int i = 0 ; i < dim ; ++i)
	rstate->values[i] =
	    rng_.uniformReal(std::max(bounds.low[i], rnear->values[i] - distance), 
			     std::min(bounds.high[i], rnear->values[i] + distance));
}

void ompl::base::RealVectorStateSampler::sampleGaussian(State *state, const State *mean, const double stdDev)
{
    const unsigned int dim = manifold_->getDimension();
    const RealVectorBounds &bounds = static_cast<const RealVectorStateManifold*>(manifold_)->getBounds();

    RealVectorStateManifold::StateType *rstate = static_cast<RealVectorStateManifold::StateType*>(state);
    const RealVectorStateManifold::StateType *rmean = static_cast<const RealVectorStateManifold::StateType*>(mean);
    for (unsigned int i = 0 ; i < dim ; ++i)
    {
	double v = rng_.gaussian(rmean->values[i], stdDev);
	if (v < bounds.low[i])
	    v = bounds.low[i];
	else
	    if (v > bounds.high[i])
		v = bounds.high[i];
	rstate->values[i] = v;
    }
}

void ompl::base::RealVectorStateManifold::setup(void)
{
    StateManifold::setup();
    bounds_.check();
    
    // compute a default random projection
    if (dimension_ > 0)
    {
	double md = std::numeric_limits<double>::infinity();
	for (unsigned int i = 0 ; i < dimension_ ; ++i)
	{
	    double d = bounds_.high[i] - bounds_.low[i];
	    if (d < md)
		md = d;
	}

	if (dimension_ > 2)
	{
	    int p = std::max(2, (int)ceil(log((double)getDimension())));
	    std::vector<double> cellDims(p, md / 5.0);
	    registerDefaultProjection(ProjectionEvaluatorPtr(new RealVectorRandomLinearProjectionEvaluator(this, cellDims)));
	}
	else
	{
	    std::vector<double> cellDims(dimension_, md / 5.0);
	    registerDefaultProjection(ProjectionEvaluatorPtr(new RealVectorIdentityProjectionEvaluator(this, cellDims)));
	}
    }
}

void ompl::base::RealVectorStateManifold::setBounds(const RealVectorBounds &bounds)
{
    bounds.check();
    if (bounds.low.size() != dimension_)
	throw Exception("Bounds do not match dimension of manifold");
    bounds_ = bounds;
}

unsigned int ompl::base::RealVectorStateManifold::getDimension(void) const
{
    return dimension_;
}

const std::string& ompl::base::RealVectorStateManifold::getDimensionName(unsigned int index) const
{
    if (index < dimensionNames_.size())
	return dimensionNames_[index];
    throw Exception("Index out of bounds");
}

int ompl::base::RealVectorStateManifold::getDimensionIndex(const std::string &name) const
{
    std::map<std::string, unsigned int>::const_iterator it = dimensionIndex_.find(name);
    return it != dimensionIndex_.end() ? it->second : -1;
}

void ompl::base::RealVectorStateManifold::setDimensionName(unsigned int index, const std::string &name)
{
    if (index < dimensionNames_.size())
	dimensionNames_[index] = name;
    else
	throw Exception("Cannot set dimension name. Index out of bounds");
}

double ompl::base::RealVectorStateManifold::getMaximumExtent(void) const
{
    double e = 0.0;
    for (unsigned int i = 0 ; i < dimension_ ; ++i)
    {
	double d = bounds_.high[i] - bounds_.low[i];
	e += d*d;
    }
    return sqrt(e);
}

void ompl::base::RealVectorStateManifold::enforceBounds(State *state) const
{
    StateType *rstate = static_cast<StateType*>(state);
    for (unsigned int i = 0 ; i < dimension_ ; ++i)
    {
	if (rstate->values[i] > bounds_.high[i])
	    rstate->values[i] = bounds_.high[i];
	else
	    if (rstate->values[i] < bounds_.low[i])
		rstate->values[i] = bounds_.low[i];
    }
}    
	    	    
bool ompl::base::RealVectorStateManifold::satisfiesBounds(const State *state) const
{
    const StateType *rstate = static_cast<const StateType*>(state);    
    for (unsigned int i = 0 ; i < dimension_ ; ++i)
	if (rstate->values[i] - std::numeric_limits<double>::epsilon() > bounds_.high[i] ||
	    rstate->values[i] + std::numeric_limits<double>::epsilon() < bounds_.low[i])
	    return false;
    return true;
}

void ompl::base::RealVectorStateManifold::copyState(State *destination, const State *source) const
{
    memcpy(static_cast<StateType*>(destination)->values,
	   static_cast<const StateType*>(source)->values, stateBytes_);    
}

double ompl::base::RealVectorStateManifold::distance(const State *state1, const State *state2) const
{
    double dist = 0.0;
    const double *s1 = static_cast<const StateType*>(state1)->values;
    const double *s2 = static_cast<const StateType*>(state2)->values;
    
    for (unsigned int i = 0 ; i < dimension_ ; ++i)
    {	 
	double diff = (*s1++) - (*s2++);
	dist += diff * diff;
    }
    return sqrt(dist);
}

bool ompl::base::RealVectorStateManifold::equalStates(const State *state1, const State *state2) const
{
    const double *s1 = static_cast<const StateType*>(state1)->values;
    const double *s2 = static_cast<const StateType*>(state2)->values;
    for (unsigned int i = 0 ; i < dimension_ ; ++i)
    {	 
	double diff = (*s1++) - (*s2++);
	if (fabs(diff) > std::numeric_limits<double>::epsilon())
	    return false;
    }
    return true;
}

void ompl::base::RealVectorStateManifold::interpolate(const State *from, const State *to, const double t, State *state) const
{
    const StateType *rfrom = static_cast<const StateType*>(from);
    const StateType *rto = static_cast<const StateType*>(to);
    const StateType *rstate = static_cast<StateType*>(state);
    for (unsigned int i = 0 ; i < dimension_ ; ++i)
	rstate->values[i] = rfrom->values[i] + (rto->values[i] - rfrom->values[i]) * t;
}

ompl::base::ManifoldStateSamplerPtr ompl::base::RealVectorStateManifold::allocStateSampler(void) const 
{
    return ManifoldStateSamplerPtr(new RealVectorStateSampler(this));
}

ompl::base::State* ompl::base::RealVectorStateManifold::allocState(void) const
{
    StateType *rstate = new StateType();
    rstate->values = new double[dimension_];
    return rstate;
}

void ompl::base::RealVectorStateManifold::freeState(State *state) const
{
    StateType *rstate = static_cast<StateType*>(state);
    delete[] rstate->values;
    delete rstate;
}

void ompl::base::RealVectorStateManifold::printState(const State *state, std::ostream &out) const
{
    out << "RealVectorState ["; 
    if (state)
    {
	const StateType *rstate = static_cast<const StateType*>(state);
        for (unsigned int i = 0 ; i < dimension_ ; ++i)
	{
	    out << rstate->values[i];
	    if (i + 1 < dimension_) 
		out << ' ';
	}
    }
    else
	out << "NULL" << std::endl;
    out << ']' << std::endl;
}

void ompl::base::RealVectorStateManifold::printSettings(std::ostream &out) const
{
    out << "Real vector state manifold '" << name_ << "' of dimension " << dimension_ << " with bounds: " << std::endl;
    out << "  - min: ";
    for (unsigned int i = 0 ; i < dimension_ ; ++i)
	out << bounds_.low[i] << " ";
    out << std::endl;    
    out << "  - max: ";
    for (unsigned int i = 0 ; i < dimension_ ; ++i)
	out << bounds_.high[i] << " ";
    out << std::endl;
}
