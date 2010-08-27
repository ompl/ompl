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

ompl::base::SpaceInformation::SpaceInformation(const StateManifoldPtr &manifold) : stateManifold_(manifold), resolution_(0.0), maxExtent_(0.0), maxResolution_(0.0), setup_(false), msg_("SpaceInformation")
{
    if (!stateManifold_)
	throw Exception("Invalid manifold definition");
}

void ompl::base::SpaceInformation::setup(void)
{
    if (setup_)
	msg_.warn("Space information setup called multiple times");
    
    if (!stateValidityChecker_)
	throw Exception("State validity checker not set!");
        
    stateManifold_->setup();
    if (stateManifold_->getDimension() <= 0)
	throw Exception("The dimension of the state manifold we plan in must be > 0");

    if (resolution_ < std::numeric_limits<double>::epsilon())
    {
	resolution_ = estimateMaxResolution(1000);
	msg_.inform("The resolution at which states need to be checked for collision is detected to be %f", resolution_);
    }

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

double ompl::base::SpaceInformation::estimateExtent(unsigned int samples)
{
    if (maxExtent_ > std::numeric_limits<double>::epsilon())
	return maxExtent_;
    
    if (samples < 2)
	samples = 2;

    // sample some states
    ManifoldStateSamplerPtr ss = allocManifoldStateSampler();
    std::vector<State*> states(samples);
    for (unsigned int i = 0 ; i  < samples ; ++i)
    {
	states[i] = allocState();
	ss->sampleUniform(states[i]);
    }
    // find pair with maximum distance
    State *a = states[0];
    State *b = states[1];
    double maxD = distance(a, b);
    
    for (unsigned int j = 0 ; j < samples ; ++j)
    {
	bool found = false;
	for (unsigned int i = 0 ; i < samples ; ++i)
	{
	    if (states[i] == a || states[i] == b)
		continue;
	    
	    double d = distance(a, states[i]);
	    if (d > maxD)
	    {
		b = states[i];
		maxD = d;
		found = true;
	    }
	}
	if (!found)
	    break;
	std::swap(a, b);
    }
        
    // free memory
    for (unsigned int i = 0 ; i  < samples ; ++i)
	freeState(states[i]);
    maxExtent_ = maxD;
    
    msg_.inform("Estimated extent of space to plan in is %f", maxD);
    if (maxD < std::numeric_limits<double>::epsilon())
	throw Exception("Estimated extent of space is too small");
    
    return maxD;
}


namespace ompl
{
    namespace base
    {
	
	// find a motion that has two valid endpoints but contains an invalid state
	static bool findCollidingMotion(State *s1, State *s2, State *invalid,
					SpaceInformation *si, ManifoldStateSampler *ss, unsigned int samples, double distance)
	{
	    // find an invalid state
	    bool found = false;
	    for (unsigned int i = 0 ; !found && i < samples ; ++i)
	    {
		ss->sampleUniform(invalid);
		found = !si->isValid(invalid);
	    }
	    if (!found)
		return false;
	    
	    // find two valid states around the invalid one
	    bool v = si->searchValidNearby(s1, invalid, distance, samples);
	    if (v)
		return si->searchValidNearby(s2, invalid, distance, samples);
	    else
		return false;
	}
	
	// given a motion defined by two states, find the length of the segment that is in collision 
	static double getCollisionLength(SpaceInformation *si, double resolution, const State *s1, const State *s2,
					 State *dummy1, State *dummy2)
	{
	    double backup = si->getStateValidityCheckingResolution();
	    si->setStateValidityCheckingResolution(resolution);
	    
	    std::pair<State*, double> lastValid1; lastValid1.first = dummy1;
	    std::pair<State*, double> lastValid2; lastValid2.first = dummy2;
	    bool v1 = si->checkMotion(s1, s2, lastValid1);
	    bool v2 = si->checkMotion(s2, s1, lastValid2);
	    
	    si->setStateValidityCheckingResolution(backup);
	    
	    return (v1 || v2) ? 0.0 : si->distance(dummy1, dummy2);
	}	
    }
}

double ompl::base::SpaceInformation::estimateMaxResolution(unsigned int samples)
{
    if (maxResolution_ > std::numeric_limits<double>::epsilon())
	return maxResolution_;
    
    if (samples < 2)
	samples = 2;

    double extent = estimateExtent(samples);
    maxResolution_ = extent / 50.0;

    msg_.debug("Initial estimate for state validity checking resolution is %f", maxResolution_);
    
    // if there are some invalid states, we can try to improve this
    // resolution.  

    // allocate a state sampler
    ManifoldStateSamplerPtr ss = allocManifoldStateSampler();

    // allocate needed temporary memory
    State *invalid = allocState();
    State *s1 = allocState();
    State *s2 = allocState();
    State *dummy = allocState();

    // array that will hold lengths of invalid segments
    std::vector<double> cl;
    
    // decide how many such segments to attempt to detect
    unsigned int steps = samples/100;
    if (steps < 10)
	steps = 10;
    if (steps > samples)
	steps = samples;

    // while we can find invalid segments, remember the length of their invalid part
    while (cl.size() < steps && findCollidingMotion(s1, s2, invalid, this, ss.get(), samples, maxResolution_ * 5.0))
    {
	double r = std::min(maxResolution_, std::min(distance(s1, invalid), distance(s2, invalid)) / 5.0);
	double l = getCollisionLength(this, r, s1, s2, invalid, dummy);
	if (l > std::numeric_limits<double>::epsilon())
	{
	    cl.push_back(l);
	    msg_.debug("Found invalid segment of length %f", l);
	}
    }
    
    if (!cl.empty())
    {
	// see if 1/3 * median(lengths of invalid segments) is a finer resolution than the one initially assumed
	std::sort(cl.begin(), cl.end());
	maxResolution_ = std::min(cl[cl.size()/2] / 3.0, maxResolution_);	
    }
    
    freeState(dummy);
    freeState(s1);
    freeState(s2);
    freeState(invalid);

    msg_.debug("Estimated state validity checking resolution is %f", maxResolution_);

    return maxResolution_;
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
    int nd = (int)ceil(distance(s1, s2) / resolution_);
    
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
    int nd = (int)ceil(distance(s1, s2) / resolution_);
    
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
    int nd = (int)ceil(distance(s1, s2) / (resolution_ * factor));
    
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
    out << "  - state validity check resolution: " << resolution_ << std::endl;
    out << "  - state manifold:" << std::endl;
    stateManifold_->printSettings(out);
}
