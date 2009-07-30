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

#include "ompl/extension/kinematic/extension/ik/HCIK.h"
#include <algorithm>

bool ompl::kinematic::HCIK::tryToImprove(base::State *state, double add, double *distance) const
{
    base::GoalRegion *goal_r = dynamic_cast<base::GoalRegion*>(m_si->getGoal());
    unsigned int   dim = m_si->getStateDimension();
    
    if (!goal_r)
	return false;
    
    double tempDistance;
    double initialDistance;

    bool wasValid = valid(state);
    bool wasValidStart = wasValid;
    
    bool wasSatisfied = goal_r->isSatisfied(state, &initialDistance);
    bool wasSatisfiedStart = wasSatisfied;
    
    double bestDist   = initialDistance;
    
    bool change = true;
    unsigned int steps = 0;
    
    while (change && steps < m_maxImproveSteps)
    {
	change = false;
	steps++;
	
	for (int i = dim - 1 ; i >= 0 ; --i)
	{
	    bool better = true;
	    bool increased = false;
	    while (better)
	    {
		better = false;
		double backup = state->values[i];
		state->values[i] += add;

		// if we are still within bounds, go on
		if (m_si->satisfiesBounds(state))
		{
		    bool isV = valid(state);
		    bool isS = goal_r->isSatisfied(state, &tempDistance);

		    if (isV && !wasValid)
		    {
			wasValid = true;
			wasSatisfied = isS;
			better = true;
			change = true;
			increased = true;
			bestDist = tempDistance;
		    }
		    else
		    {
			if (isV == wasValid)
			{			    
			    // if this change made the goal satisfied, definitely keep it
			    if (isS && !wasSatisfied)
			    {	
				wasSatisfied = true;
				better = true;
				change = true;
				increased = true;
				bestDist = tempDistance;
			    }
			    else
			    {
				// if we are at least not going to an unsatisfied state
				if (isS == wasSatisfied)
				{
				    // and we are improving
				    if (tempDistance < bestDist)
				    {
					better = true;
					change = true;
					increased = true;
					bestDist = tempDistance;
				    } 
				    else
					state->values[i] = backup;
				}
				else
				    state->values[i] = backup;
			    }
			}
			else
			    state->values[i] = backup;
		    }
		}
		else
		    state->values[i] = backup;
	    }
	    
	    // if increasing did not help, maybe decreasing will
	    if (!increased)
	    {
		better = true;
		while (better)
		{
		    better = false;
		    double backup = state->values[i];
		    state->values[i] -= add;
		    
		    // if we are still within bounds, go on
		    if (m_si->satisfiesBounds(state))
		    {	
			bool isV = valid(state);
			bool isS = goal_r->isSatisfied(state, &tempDistance);

			if (isV && !wasValid)
			{
			    wasValid = true;
			    wasSatisfied = isS;
			    better = true;
			    change = true;
			    bestDist = tempDistance;
			}
			else
			{
			    if (isV == wasValid)
			    {
				// if this change made the goal satisfied, definitely keep it
				if (isS && !wasSatisfied)
				{	
				    wasSatisfied = true;
				    better = true;
				    change = true;
				    bestDist = tempDistance;
				}
				else
				{
				    // if we are at least not going to an unsatisfied state
				    if (isS == wasSatisfied)
				    {
					// and we are improving
					if (tempDistance < bestDist)
					{
					    better = true;
					    change = true;
					    bestDist = tempDistance;
					} 
					else
					    state->values[i] = backup;
				    }
				    else
					state->values[i] = backup;
				}
			    }
			    else
				state->values[i] = backup;
			}
		    }
		    else
			state->values[i] = backup;
		}
	    }
	}
    }
    
    
    if (distance)
	*distance = bestDist;
    return (bestDist < initialDistance) || (!wasSatisfiedStart && wasSatisfied) || (!wasValidStart && wasValid);
}
