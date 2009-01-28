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
	    if (si->isValid(static_cast<SpaceInformationKinematic::StateKinematic_t>(pool[i].state)))
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
	for (unsigned int i = 0 ; i < 5 ; ++i)
	{
	    if (si->isValid(static_cast<SpaceInformationKinematic::StateKinematic_t>(pool[i].state)))
	    {
		if (tryToSolve(pool[i].state, &(pool[i].distance)))
		{
		    solved = true;
		    solution = i;
		    break;
		}
	    }
	}
	for (unsigned int i = m_poolSize ; i < maxPoolSize ; ++i)
	{
	    si->sampleNear(pool[i].state, pool[i%m_poolSize].state, range);
	    if (goal_r->isSatisfied(pool[i].state, &(pool[i].distance)))
	    {
		if (si->isValid(static_cast<SpaceInformationKinematic::StateKinematic_t>(pool[i].state)))
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
	SpaceInformationKinematic::PathKinematic_t path = new SpaceInformationKinematic::PathKinematic(m_si);
	SpaceInformationKinematic::StateKinematic_t st = new SpaceInformationKinematic::StateKinematic(dim);
	si->copyState(st, pool[solution].state);
	path->states.push_back(st);
	goal_r->setDifference(pool[solution].distance);
	goal_r->setSolutionPath(path);
    }
    else
    {	
	/* find an approximate solution */
	std::sort(pool.begin(), pool.end(), gs);
	for (unsigned int i = 0 ; i < 5 ; ++i)
	{	
	    if (si->isValid(static_cast<SpaceInformationKinematic::StateKinematic_t>(pool[i].state)))
	    {
		SpaceInformationKinematic::PathKinematic_t path = new SpaceInformationKinematic::PathKinematic(m_si);
		SpaceInformationKinematic::StateKinematic_t st = new SpaceInformationKinematic::StateKinematic(dim);
		si->copyState(st, pool[i].state);
		path->states.push_back(st);
		goal_r->setDifference(pool[i].distance);
		goal_r->setSolutionPath(path, true);
		break;
	    }
	}
    }
    
    for (unsigned int i = 0 ; i < maxPoolSize ; ++i)
	delete pool[i].state;
    
    return goal_r->isAchieved();
}

bool ompl::GAIK::tryToSolve(SpaceInformationKinematic::StateKinematic_t state, double *distance)
{
    double dist = *distance;
    
    if (tryToSolveFact(1.01, state, distance))
	return true;
    
    if (dist > *distance)
    {
	dist = *distance;
	if (tryToSolveFact(1.003, state, distance))
	    return true;
    }
    else
	return false;
    
    if (dist > *distance)
    {
	dist = *distance;
	if (tryToSolveFact(1.001, state, distance))
	    return true;
    }
    
    return false;
}

bool ompl::GAIK::tryToSolveFact(double factorP, SpaceInformationKinematic::StateKinematic_t state, double *distance)
{
    SpaceInformationKinematic_t                          si = dynamic_cast<SpaceInformationKinematic_t>(m_si); 
    SpaceInformationKinematic::GoalRegionKinematic_t goal_r = dynamic_cast<SpaceInformationKinematic::GoalRegionKinematic_t>(si->getGoal());
    unsigned int                                        dim = si->getStateDimension();
    
    double factorM = 1.0/factorP;
    
    bool wasSatisfied = goal_r->isSatisfied(state, distance);
    double bestDist   = *distance;
    
    bool change = true;
    
    while (change)
    {
	change = false;
	
	for (unsigned int i = 0 ; i < dim ; ++i)
	{
	    bool better = true;
	    while (better)
	    {
		better = false;
		double backup = state->values[i];
		state->values[i] *= factorP;
		bool isS = goal_r->isSatisfied(state, distance);
		if (wasSatisfied && !isS)
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
	    
	    better = true;
	    while (better)
	    {
		better = false;
		double backup = state->values[i];
		state->values[i] *= factorM;
		bool isS = goal_r->isSatisfied(state, distance);
		if (wasSatisfied && !isS)
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
    
    *distance = bestDist;
    return wasSatisfied && si->isValid(static_cast<SpaceInformationKinematic::StateKinematic_t>(state));
}
