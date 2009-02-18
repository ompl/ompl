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
    return m_checkValidity ? m_si->isValid(static_cast<SpaceInformationKinematic::StateKinematic_t>(state)) : true;
}

bool ompl::GAIK::solve(double solveTime)
{
    SpaceInformationKinematic_t                          si = dynamic_cast<SpaceInformationKinematic_t>(m_si); 
    SpaceInformationKinematic::GoalRegionKinematic_t goal_r = dynamic_cast<SpaceInformationKinematic::GoalRegionKinematic_t>(si->getGoal());
    unsigned int                                        dim = si->getStateDimension();
    
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
    
    for (unsigned int i = 0 ; i < maxPoolSize ; ++i)
    {
	pool[i].state = new SpaceInformationKinematic::StateKinematic(dim);
	si->sample(pool[i].state);
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
	range[i] = m_rho * (si->getStateComponent(i).maxValue - si->getStateComponent(i).minValue);
    
    while (!solved && time_utils::Time::now() < endTime)
    {
	generations++;
	std::sort(pool.begin(), pool.end(), gs);
	
	for (unsigned int i = m_poolSize ; i < maxPoolSize ; ++i)
	{
	    si->sampleNear(pool[i].state, pool[i%m_poolSize].state, range);
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
	SpaceInformationKinematic::PathKinematic_t path = new SpaceInformationKinematic::PathKinematic(m_si);
	SpaceInformationKinematic::StateKinematic_t st = new SpaceInformationKinematic::StateKinematic(dim);
	si->copyState(st, pool[solution].state);
	path->states.push_back(st);
	goal_r->setDifference(pool[solution].distance);
	goal_r->setSolutionPath(path);

	// try to improve the solution
	double dist = goal_r->getDifference();
	tryToImprove(st, &dist);
	goal_r->setDifference(dist);

	// if improving the state made it invalid, revert
	if (!valid(st))
	{
	    si->copyState(st, pool[solution].state);
	    goal_r->setDifference(pool[solution].distance);
	}
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
		SpaceInformationKinematic::PathKinematic_t path = new SpaceInformationKinematic::PathKinematic(m_si);
		SpaceInformationKinematic::StateKinematic_t st = new SpaceInformationKinematic::StateKinematic(dim);
		si->copyState(st, pool[i].state);
		path->states.push_back(st);

		// try to improve the state
		double dist = pool[i].distance;
		tryToImprove(st, &dist);
		
		// if the improved state is still valid, we have a solution
		if (valid(st))
		{
		    // the solution may no longer be just approximate, so we check
		    bool approx = goal_r->isSatisfied(st, &dist);
		    goal_r->setSolutionPath(path, approx);
		    goal_r->setDifference(dist);
		}
		else
		{
		    // if the improvement made the state no longer valid, revert to previous one
		    si->copyState(st, pool[i].state);
		    goal_r->setSolutionPath(path, true);
		    goal_r->setDifference(pool[i].distance);
		}
		break;
	    }
	}
    }
    
    for (unsigned int i = 0 ; i < maxPoolSize ; ++i)
	delete pool[i].state;
    
    return goal_r->isAchieved();
}

bool ompl::GAIK::tryToImprove(SpaceInformationKinematic::StateKinematic_t state, double *distance)
{
    m_msg.inform("GAIK: Distance to goal before improvement: %g", *distance);    
    time_utils::Time start = time_utils::Time::now();
    tryToImproveAux(0.1, state, distance);
    tryToImproveAux(0.05, state, distance);
    tryToImproveAux(0.01, state, distance);
    tryToImproveAux(0.005, state, distance);
    tryToImproveAux(0.001, state, distance);
    tryToImproveAux(0.0005, state, distance);
    tryToImproveAux(0.0001, state, distance);
    tryToImproveAux(0.00005, state, distance);
    tryToImproveAux(0.000025, state, distance);
    tryToImproveAux(0.00001, state, distance);
    tryToImproveAux(0.000005, state, distance);
    m_msg.inform("GAIK: Improvement took  %g seconds", (time_utils::Time::now() - start).toSeconds());    
    m_msg.inform("GAIK: Distance to goal after improvement: %g", *distance);    
    return true;
}

bool ompl::GAIK::tryToImproveAux(double add, SpaceInformationKinematic::StateKinematic_t state, double *distance)
{
    SpaceInformationKinematic_t                          si = dynamic_cast<SpaceInformationKinematic_t>(m_si); 
    SpaceInformationKinematic::GoalRegionKinematic_t goal_r = dynamic_cast<SpaceInformationKinematic::GoalRegionKinematic_t>(si->getGoal());
    unsigned int                                        dim = si->getStateDimension();
    
    bool wasSatisfied = goal_r->isSatisfied(state, distance);
    double bestDist   = *distance;
    
    bool change = true;
    unsigned int steps = 0;
    
    while (change && steps < m_maxImproveSteps)
    {
	change = false;
	steps++;
	
	for (unsigned int i = 0 ; i < dim ; ++i)
	{
	    bool better = true;
	    bool increased = false;
	    while (better)
	    {
		better = false;
		double backup = state->values[i];
		state->values[i] += add;
		bool isS = goal_r->isSatisfied(state, distance);
		if ((wasSatisfied && !isS) || !si->satisfiesBounds(state))
		    state->values[i] = backup;
		else
		{
		    wasSatisfied = isS;
		    if (*distance < bestDist)
		    {
			better = true;
			change = true;
			increased = true;
			bestDist = *distance;
		    } 
		    else
			state->values[i] = backup;
		}
	    }

	    if (!increased)
	    {
		better = true;
		while (better)
		{
		    better = false;
		    double backup = state->values[i];
		    state->values[i] -= add;
		    bool isS = goal_r->isSatisfied(state, distance);
		    if ((wasSatisfied && !isS) || !si->satisfiesBounds(state))
			state->values[i] = backup;
		    else
		    {
			wasSatisfied = isS;
			if (*distance < bestDist)
			{
			    better = true;
			    change = true;
			    bestDist = *distance;
			} 
			else
			    state->values[i] = backup;
		    }
		}
	    }
	}
    }
    
    *distance = bestDist;
    return wasSatisfied && valid(state);
}
