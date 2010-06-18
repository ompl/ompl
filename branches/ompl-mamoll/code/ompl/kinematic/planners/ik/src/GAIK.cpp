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

#include "ompl/kinematic/planners/ik/GAIK.h"
#include "ompl/base/GoalRegion.h"

#include "ompl/util/Time.h"

#include <algorithm>

bool ompl::kinematic::GAIK::solve(double solveTime, const base::GoalRegion *goal, base::State *result, const std::vector<base::State*> &hint)
{
    unsigned int            dim = m_si->getStateDimension();
    time::point             endTime = time::now() + time::seconds(solveTime);
    
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
    
    std::vector<double> range(dim);    
    for (unsigned int i = 0 ; i < dim ; ++i)
	range[i] = m_rho * (m_si->getStateComponent(i).maxValue - m_si->getStateComponent(i).minValue);
    
    // add hint states
    unsigned int nh = std::min<unsigned int>(maxPoolSize, hint.size());
    for (unsigned int i = 0 ; i < nh ; ++i)
    {
	pool[i].state = new base::State(dim);
	m_si->copyState(pool[i].state, hint[i]);
	if (!m_si->satisfiesBounds(pool[i].state))
	    m_si->enforceBounds(pool[i].state);
	pool[i].valid = valid(pool[i].state);
	if (goal->isSatisfied(pool[i].state, &(pool[i].distance)))
	{
	    if (pool[i].valid)
	    {
		solved = true;
		solution = i;
	    }
	}
    }
    
    // add states near the hint states
    unsigned int nh2 = nh * 2;    
    if (nh2 < maxPoolSize)
    {
	for (unsigned int i = nh ; i < nh2 ; ++i)
	{
	    pool[i].state = new base::State(dim);
	    m_sCore->sampleNear(pool[i].state, pool[i % nh].state, range);
	    pool[i].valid = valid(pool[i].state);
	    if (goal->isSatisfied(pool[i].state, &(pool[i].distance)))
	    {
		if (pool[i].valid)
		{
		    solved = true;
		    solution = i;
		}
	    }
	}
	nh = nh2;
    }
    
    // add random states
    for (unsigned int i = nh ; i < maxPoolSize ; ++i)
    {
	pool[i].state = new base::State(dim);
	m_sCore->sample(pool[i].state);
	pool[i].valid = valid(pool[i].state);
	if (goal->isSatisfied(pool[i].state, &(pool[i].distance)))
	{
	    if (pool[i].valid)
	    {
		solved = true;
		solution = i;
	    }
	}
    }
    
    // run the genetic algorithm
    unsigned int generations = 1;
    unsigned int mutationsSize = maxPoolSize - maxPoolSize % m_poolSize;
    if (mutationsSize == 0)
	mutationsSize = maxPoolSize;
    if (mutationsSize == maxPoolSize)
	mutationsSize--;
    
    while (!solved && time::now() < endTime)
    {
	generations++;
	std::sort(pool.begin(), pool.end(), gs);
	
	// add mutations
	for (unsigned int i = m_poolSize ; i < mutationsSize ; ++i)
	{
	    m_sCore->sampleNear(pool[i].state, pool[i % m_poolSize].state, range);
	    pool[i].valid = valid(pool[i].state);
	    if (goal->isSatisfied(pool[i].state, &(pool[i].distance)))
	    {
		if (pool[i].valid)
		{
		    solved = true;
		    solution = i;
		    break;
		}
	    }
	}

	// add random states
	if (!solved)
	    for (unsigned int i = mutationsSize ; i < maxPoolSize ; ++i)
	    {
		m_sCore->sample(pool[i].state);
		pool[i].valid = valid(pool[i].state);
		if (goal->isSatisfied(pool[i].state, &(pool[i].distance)))
		{
		    if (pool[i].valid)
		    {
			solved = true;
			solution = i;
			break;
		    }
		}
	    }
    }
    
    
    // fill in solution, if found
    m_msg.inform("GAIK: Ran for %u generations", generations);

    if (solved)
    {
	// set the solution 
	m_si->copyState(result, pool[solution].state);

	// try to improve the solution
	tryToImprove(goal, result, pool[solution].distance);

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
	    if (pool[i].valid)
	    {
		// set the solution
		m_si->copyState(result, pool[i].state);

		// try to improve the state
		tryToImprove(goal, result, pool[i].distance);
		
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

bool ompl::kinematic::GAIK::tryToImprove(const base::GoalRegion *goal, base::State *state, double distance)
{
    m_msg.debug("GAIK: Distance to goal before improvement: %g", distance);    
    time::point start = time::now();
    m_hcik.tryToImprove(goal, state, 0.1, &distance);
    m_hcik.tryToImprove(goal, state, 0.05, &distance);
    m_hcik.tryToImprove(goal, state, 0.01, &distance);
    m_hcik.tryToImprove(goal, state, 0.005, &distance);
    m_hcik.tryToImprove(goal, state, 0.001, &distance);
    m_hcik.tryToImprove(goal, state, 0.005, &distance);
    m_hcik.tryToImprove(goal, state, 0.0001, &distance);
    m_hcik.tryToImprove(goal, state, 0.00005, &distance);
    m_hcik.tryToImprove(goal, state, 0.000025, &distance);
    m_hcik.tryToImprove(goal, state, 0.000005, &distance);
    m_msg.debug("GAIK: Improvement took  %g ms", (time::now() - start).total_milliseconds());
    m_msg.debug("GAIK: Distance to goal after improvement: %g", distance);
    return true;
}
