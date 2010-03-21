/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
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

/* \author Ioan Sucan */

#include "ompl/kinematic/LinearStateInterpolatorKinematic.h"
#include <algorithm>
#include <valarray>
#include <queue>

bool ompl::kinematic::LinearStateInterpolatorKinematic::checkMotion(const base::State *s1, const base::State *s2,
								    base::State *lastValidState, double *lastValidTime) const
{
    return (lastValidState == NULL && lastValidTime == NULL) ?
	subdivisionCheck(s1, s2) : incrementalCheck(s1, s2, lastValidState, lastValidTime);
}


namespace ompl
{
    namespace kinematic
    {
	
	/** \brief For functions that need to interpolate between two states, find the appropriate step size */
	static int findDifferenceStep(const base::State *s1, const base::State *s2, double factor,
				      const std::vector<base::StateComponent> &stateComponent,
				      const unsigned int stateDimension,
				      std::valarray<double> &step)
	{
	    /* find diffs */
	    std::valarray<double> diff(stateDimension);
	    for (unsigned int i = 0 ; i < stateDimension ; ++i)
		diff[i] = s2->values[i] - s1->values[i];
	    
	    // will need to handle quaternions; use bullet LinearMath? use slerp?
	    
	    /* find out how many divisions we need */
	    int nd = 1;
	    for (unsigned int i = 0 ; i < stateDimension ; ++i)
	    {
		int d = 1 + (int)(fabs(diff[i]) / (factor * stateComponent[i].resolution));
		if (nd < d)
		    nd = d;
	    }
	    
	    /* find out the step size as a vector */
	    step.resize(stateDimension); 
	    step = diff / (double)nd;
	    
	    return nd;
	}
	
    }
}

bool ompl::kinematic::LinearStateInterpolatorKinematic::subdivisionCheck(const base::State *s1, const base::State *s2) const
{
    /* assume motion starts in a valid configuration so s1 is valid */
    if (!m_si->isValid(s2))
	return false;
    
    std::valarray<double> step;
    const unsigned int sDim = m_si->getStateDimension();
    int nd = findDifferenceStep(s1, s2, 1.0, m_si->getStateComponents(), sDim, step);
    
    /* initialize the queue of test positions */
    std::queue< std::pair<int, int> > pos;
    if (nd >= 2)
	pos.push(std::make_pair(1, nd - 1));
    
    /* temporary storage for the checked state */
    base::State test(sDim);
    
    /* repeatedly subdivide the path segment in the middle (and check the middle) */
    while (!pos.empty())
    {
	std::pair<int, int> x = pos.front();
	
	int mid = (x.first + x.second) / 2;
	
	for (unsigned int j = 0 ; j < sDim ; ++j)
	    test.values[j] = s1->values[j] + (double)mid * step[j];
	
	if (!m_si->isValid(&test))
	    return false;
	
	pos.pop();
	
	if (x.first < mid)
	    pos.push(std::make_pair(x.first, mid - 1));
	if (x.second > mid)
	    pos.push(std::make_pair(mid + 1, x.second));
    }
    
    return true;
}

bool ompl::kinematic::LinearStateInterpolatorKinematic::incrementalCheck(const base::State *s1, const base::State *s2,
									 base::State *lastValidState, double *lastValidTime) const
{   
    /* assume motion starts in a valid configuration so s1 is valid */
    if (!m_si->isValid(s2))
	return false;
    
    std::valarray<double> step;
    const unsigned int sDim = m_si->getStateDimension();
    int nd = findDifferenceStep(s1, s2, 1.0, m_si->getStateComponents(), sDim, step);
    
    /* temporary storage for the checked state */
    base::State test(sDim);
    
    for (int j = 1 ; j < nd ; ++j)
    {
	double factor = (double)j;
	for (unsigned int k = 0 ; k < sDim ; ++k)
	    test.values[k] = s1->values[k] + factor * step[k];
	if (!m_si->isValid(&test))
	{
	    if (lastValidState)
	    {
		factor -= 1.0;
		for (unsigned int k = 0 ; k < sDim ; ++k)
		    lastValidState->values[k] = s1->values[k] + factor * step[k];
	    }
	    if (lastValidTime)
		*lastValidTime = (double)(j - 1) / (double)nd;
	    return false;
	}
    }
    
    return true;
}

unsigned int ompl::kinematic::LinearStateInterpolatorKinematic::getStates(const base::State *s1, const base::State *s2,
									  std::vector<base::State*> &states,
									  double factor, bool endpoints, bool alloc) const
{
    std::valarray<double> step;
    const unsigned int sDim = m_si->getStateDimension();
    int nd = findDifferenceStep(s1, s2, factor, m_si->getStateComponents(), sDim, step);
    
    if (alloc)
    {
	states.resize(nd + (endpoints ? 1 : -1));
	if (endpoints)
	    states[0] = new base::State(sDim);
    }
    
    unsigned int added = 0;
    
    if (endpoints && states.size() > 0)
    {
	m_si->copyState(states[0], s1);
	added++;
    }
    
    /* find the states in between */
    for (int j = 1 ; j < nd && added < states.size() ; ++j)
    {
	if (alloc)
	    states[added] = new base::State(sDim);
	base::State *state = states[added];
	for (unsigned int k = 0 ; k < sDim ; ++k)
	    state->values[k] = s1->values[k] + (double)j * step[k];
	added++;
    }
    
    if (added < states.size() && endpoints)
    {
	if (alloc)
	    states[added] = new base::State(sDim);
	m_si->copyState(states[added], s2);
	added++;
    }
    
    return added;
}

