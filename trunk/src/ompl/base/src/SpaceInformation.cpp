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

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/samplers/UniformValidStateSampler.h"
#include "ompl/util/Exception.h"
#include <queue>
#include <algorithm>
#include <limits>
#include <cmath>
#include <cassert>

ompl::base::SpaceInformation::SpaceInformation(const StateManifoldPtr &manifold) : 
    sa_(manifold), stateManifold_(manifold), resolution_(0.01), setup_(false), msg_("SpaceInformation")
{
    if (!stateManifold_)
	throw Exception("Invalid manifold definition");
}

void ompl::base::SpaceInformation::setup(void)
{
    if (!stateValidityChecker_)
	throw Exception("State validity checker not set!");
        
    stateManifold_->setup();
    if (stateManifold_->getDimension() <= 0)
	throw Exception("The dimension of the state manifold we plan in must be > 0");

    if (resolution_ < std::numeric_limits<double>::epsilon() || resolution_ > 1.0 - std::numeric_limits<double>::epsilon())
	throw Exception("The specified resolution at which states need to be checked for validity must be larger than 0 and less than 1");
    
    setup_ = true;
}

bool ompl::base::SpaceInformation::isSetup(void) const
{
    return setup_;
}

void ompl::base::SpaceInformation::setStateValidityChecker(const StateValidityCheckerFn &svc)
{
    class BoostFnStateValidityChecker : public StateValidityChecker
    {
    public:
	
	BoostFnStateValidityChecker(SpaceInformation* si,
				    const StateValidityCheckerFn &fn) : StateValidityChecker(si), fn_(fn)
	{
	}
	
	virtual bool isValid(const State *state) const
	{
	    return fn_(state);
	}

    protected:

	StateValidityCheckerFn fn_;	
    };
    
    if (!svc)
	throw Exception("Invalid function definition for state validity checking");
    
    setStateValidityChecker(StateValidityCheckerPtr(dynamic_cast<StateValidityChecker*>(new BoostFnStateValidityChecker(this, svc))));
}

bool ompl::base::SpaceInformation::searchValidNearby(State *state, const State *near, double distance, unsigned int attempts) const
{
    if (state != near)
	copyState(state, near);
    
    // fix bounds, if needed
    if (!satisfiesBounds(state))
	enforceBounds(state);
    
    bool result = isValid(state);
    
    if (!result)
    {
	// try to find a valid state nearby
	ManifoldStateSamplerPtr ss = allocManifoldStateSampler();
	State        *temp = allocState();
	copyState(temp, state);	
	for (unsigned int i = 0 ; i < attempts && !result ; ++i)
	{
	    ss->sampleUniformNear(state, temp, distance);
	    result = isValid(state);
	}
	stateManifold_->freeState(temp);
    }
    
    return result;
}

bool ompl::base::SpaceInformation::checkMotion(const State *s1, const State *s2, std::pair<State*, double> &lastValid) const
{
    /* assume motion starts in a valid configuration so s1 is valid */
    if (!isValid(s2))
	return false;

    bool result = true;
    int nd = (int)ceil(stateManifold_->distanceAsFraction(s1, s2) / resolution_);
    
    /* temporary storage for the checked state */
    State *test = allocState();
    
    for (int j = 1 ; j < nd ; ++j)
    {
	stateManifold_->interpolate(s1, s2, (double)j / (double)nd, test);
	if (!isValid(test))
	{
	    if (lastValid.first)
		stateManifold_->interpolate(s1, s2, (double)(j - 1) / (double)nd, lastValid.first);
	    lastValid.second = (double)(j - 1) / (double)nd;
	    result = false;
	    break;
	}
    }
    freeState(test);
    
    return result;
}

bool ompl::base::SpaceInformation::checkMotion(const State *s1, const State *s2) const
{
    /* assume motion starts in a valid configuration so s1 is valid */
    if (!isValid(s2))
	return false;
    
    bool result = true;
    int nd = (int)ceil(stateManifold_->distanceAsFraction(s1, s2) / resolution_);
    
    /* initialize the queue of test positions */
    std::queue< std::pair<int, int> > pos;
    if (nd >= 2)
    {
	pos.push(std::make_pair(1, nd - 1));
    
	/* temporary storage for the checked state */
	State *test = allocState();
	
	/* repeatedly subdivide the path segment in the middle (and check the middle) */
	while (!pos.empty())
	{
	    std::pair<int, int> x = pos.front();
	    
	    int mid = (x.first + x.second) / 2;
	    stateManifold_->interpolate(s1, s2, (double)mid / (double)nd, test);
	    
	    if (!isValid(test))
	    {
		result = false;
		break;
	    }
	    
	    pos.pop();
	    
	    if (x.first < mid)
		pos.push(std::make_pair(x.first, mid - 1));
	    if (x.second > mid)
		pos.push(std::make_pair(mid + 1, x.second));
	}
	
	freeState(test);
    }
    
    return result;
}

unsigned int ompl::base::SpaceInformation::getMotionStates(const State *s1, const State *s2, std::vector<State*> &states, double factor, bool endpoints, bool alloc) const
{
    assert(factor > std::numeric_limits<double>::epsilon());    
    int nd = (int)ceil(stateManifold_->distanceAsFraction(s1, s2) / (resolution_ * factor));
    
    if (nd < 2)
    {
	unsigned int added = 0;
	if (endpoints)
	{
	    if (alloc)
	    {
		states.resize(2);
		states[0] = allocState();
		states[1] = allocState();
	    }
	    if (states.size() > 0)
	    {
		copyState(states[0], s1);
		added++;
	    }
	    
	    if (states.size() > 1)
	    {
		copyState(states[1], s2);
		added++;
	    }
	}
	return added;
    }
    
    if (alloc)
    {
	states.resize(nd + (endpoints ? 1 : -1));
	if (endpoints)
	    states[0] = allocState();
    }
    
    unsigned int added = 0;
    
    if (endpoints && states.size() > 0)
    {
	copyState(states[0], s1);
	added++;
    }
    
    /* find the states in between */
    for (int j = 1 ; j < nd && added < states.size() ; ++j)
    {
	if (alloc)
	    states[added] = allocState();
	stateManifold_->interpolate(s1, s2, (double)j / (double)nd, states[added]);
	added++;
    }
    
    if (added < states.size() && endpoints)
    {
	if (alloc)
	    states[added] = allocState();
	copyState(states[added], s2);
	added++;
    }
    
    return added;
}


bool ompl::base::SpaceInformation::checkMotion(const std::vector<State*> &states, unsigned int count, unsigned int &firstInvalidStateIndex) const
{
    assert(states.size() >= count);
    for (unsigned int i = 0 ; i < count ; ++i)
	if (!isValid(states[i]))
	{
	    firstInvalidStateIndex = i;
	    return false;
	}
    return true;
}

bool ompl::base::SpaceInformation::checkMotion(const std::vector<State*> &states, unsigned int count) const
{ 
    assert(states.size() >= count);
    if (count > 0)
    {
	if (count > 1)
	{
	    if (!isValid(states.front()))
		return false;
	    if (!isValid(states[count - 1]))
		return false;
	    
	    // we have 2 or more states, and the first and last states are valid
	    
	    if (count > 2)
	    {
		std::queue< std::pair<int, int> > pos;
		pos.push(std::make_pair(0, count - 1));
	    
		while (!pos.empty())
		{
		    std::pair<int, int> x = pos.front();
		    
		    int mid = (x.first + x.second) / 2;
		    if (!isValid(states[mid]))
			return false;

		    if (x.first < mid - 1)
			pos.push(std::make_pair(x.first, mid));
		    if (x.second > mid + 1)
			pos.push(std::make_pair(mid, x.second));
		}
	    }
	}
	else
	    return isValid(states.front());
    }
    return true;
}

ompl::base::ValidStateSamplerPtr ompl::base::SpaceInformation::allocValidStateSampler(void) const
{
    if (vssa_)
	return vssa_(this);
    else
	return ValidStateSamplerPtr(new UniformValidStateSampler(this));
}

void ompl::base::SpaceInformation::printSettings(std::ostream &out) const
{
    out << "State space settings:" << std::endl;
    out << "  - dimension: " << stateManifold_->getDimension() << std::endl;
    out << "  - extent: " << stateManifold_->getMaximumExtent() << std::endl;
    out << "  - state validity check resolution: " << (resolution_ * 100.0) << '%' << std::endl;
    out << "  - state manifold:" << std::endl;
    stateManifold_->printSettings(out);
}
