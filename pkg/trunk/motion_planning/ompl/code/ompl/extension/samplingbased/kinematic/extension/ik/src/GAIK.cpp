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

#include "ompl/extension/samplingbased/kinematic/extension/ik/GAIK.h"
#include <algorithm>

bool ompl::GAIK::valid(SpaceInformationKinematic::StateKinematic_t state)
{
    return m_checkValidity ? m_si->isValid(static_cast<SpaceInformation::State_t>(state)) : true;
}

bool ompl::GAIK::solve(double solveTime, SpaceInformationKinematic::StateKinematic_t result, SpaceInformationKinematic::StateKinematic_t hint)
{
    SpaceInformationKinematic::GoalRegionKinematic_t goal_r = dynamic_cast<SpaceInformationKinematic::GoalRegionKinematic_t>(m_si->getGoal());
    unsigned int                                        dim = m_si->getStateDimension();
    
    if (!goal_r)
    {
	m_msg.error("GAIK: Unknown type of goal (or goal undefined)");
	return false;
    }
    
    time_utils::Time        endTime = time_utils::Time::now() + time_utils::Duration(solveTime);
    
    unsigned int            maxPoolSize = m_poolSize + m_poolExpansion;
    std::vector<Individual> pool(maxPoolSize);
    IndividualSort          gs;
    bool                    solved = false;
    int                     solution = -1;
    
    if (m_poolSize < 1)
    {
	m_msg.error("GAIK: Pool size too small");
	return false;	
    }
    
    if (hint)
    {
	pool[0].state = new SpaceInformationKinematic::StateKinematic(dim);
	m_si->copyState(pool[0].state, hint);
	if (goal_r->isSatisfied(pool[0].state, &(pool[0].distance)))
	{
	    if (valid(pool[0].state))
	    {
		solved = true;
		solution = 0;
	    }
	}
    }
    
    for (unsigned int i = (hint ? 1 : 0) ; i < maxPoolSize ; ++i)
    {
	pool[i].state = new SpaceInformationKinematic::StateKinematic(dim);
	m_sCore.sample(pool[i].state);
	if (goal_r->isSatisfied(pool[i].state, &(pool[i].distance)))
	{
	    if (valid(pool[i].state))
	    {
		solved = true;
		solution = i;
	    }
	}
    }
    
    unsigned int generations = 1;

    std::vector<double> range(dim);    
    for (unsigned int i = 0 ; i < dim ; ++i)
	range[i] = m_rho * (m_si->getStateComponent(i).maxValue - m_si->getStateComponent(i).minValue);
    
    while (!solved && time_utils::Time::now() < endTime)
    {
	generations++;
	std::sort(pool.begin(), pool.end(), gs);
	
	for (unsigned int i = m_poolSize ; i < maxPoolSize ; ++i)
	{
	    m_sCore.sampleNear(pool[i].state, pool[i%m_poolSize].state, range);
	    if (goal_r->isSatisfied(pool[i].state, &(pool[i].distance)))
	    {
		if (valid(pool[i].state))
		{
		    solved = true;
		    solution = i;
		    break;
		}
	    }
	}
    }
    
    m_msg.inform("GAIK: Ran for %u generations", generations);

    if (solved)
    {
	// set the solution 
	m_si->copyState(result, pool[solution].state);

	// try to improve the solution
	tryToImprove(result, pool[solution].distance);

	// if improving the state made it invalid, revert
	if (!valid(result))
	    m_si->copyState(result, pool[solution].state);
    }
    else
    {	
	/* find an approximate solution */
	std::sort(pool.begin(), pool.end(), gs);
	for (unsigned int i = 0 ; i < 5 ; ++i)
	{	
	    // get a valid state that is closer to the goal
	    if (valid(pool[i].state))
	    {
		// set the solution
		m_si->copyState(result, pool[i].state);

		// try to improve the state
		tryToImprove(result, pool[i].distance);
		
		// if the improvement made the state no longer valid, revert to previous one
		if (!valid(result))		    
		    m_si->copyState(result, pool[i].state);
		solved = true;
		break;
	    }
	}
    }
    
    for (unsigned int i = 0 ; i < maxPoolSize ; ++i)
	delete pool[i].state;
    
    return solved;
}

bool ompl::GAIK::tryToImprove(SpaceInformationKinematic::StateKinematic_t state, double distance)
{
    m_msg.inform("GAIK: Distance to goal before improvement: %g", distance);    
    time_utils::Time start = time_utils::Time::now();
    m_hcik.tryToImprove(state, 0.1, &distance);
    m_hcik.tryToImprove(state, 0.05, &distance);
    m_hcik.tryToImprove(state, 0.01, &distance);
    m_hcik.tryToImprove(state, 0.005, &distance);
    m_hcik.tryToImprove(state, 0.001, &distance);
    m_hcik.tryToImprove(state, 0.005, &distance);
    m_hcik.tryToImprove(state, 0.0001, &distance);
    m_hcik.tryToImprove(state, 0.00005, &distance);
    m_hcik.tryToImprove(state, 0.000025, &distance);
    m_hcik.tryToImprove(state, 0.000005, &distance);
    m_msg.inform("GAIK: Improvement took  %g seconds", (time_utils::Time::now() - start).toSeconds());    
    m_msg.inform("GAIK: Distance to goal after improvement: %g", distance);    
    return true;
}
