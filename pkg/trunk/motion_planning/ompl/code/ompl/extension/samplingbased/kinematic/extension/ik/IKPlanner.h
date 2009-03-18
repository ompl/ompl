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

#ifndef OMPL_EXTENSION_SAMPLINGBASED_KINEMATIC_EXTENSION_IK_IKPLANNER_
#define OMPL_EXTENSION_SAMPLINGBASED_KINEMATIC_EXTENSION_IK_IKPLANNER_

#include "ompl/extension/samplingbased/kinematic/extension/ik/GAIK.h"

/** Main namespace */
namespace ompl
{

    namespace sb
    {
	
	/**
	   @subsubsection IKPlanner Inverse Kinematics Planner
	   
	   @par Short description     
	   
	   This simply runs GAIK and then the planner. This should be used
	   primarily for bi-tree planners when the specification of the
	   goal is implicit
	   
	   @par External documentation
	   
	*/
	
	template<typename _P>
	class IKPlanner : public _P
	{
	public:
	    
	    IKPlanner(SpaceInformationKinematic *si) : _P(si),
		        		               m_gaik(si)
	    {
		_P::m_type = (PlannerType)(_P::m_type | PLAN_TO_GOAL_REGION);		
	    }
	    
	    virtual ~IKPlanner(void)
	    {
	    }
	    
	    void setIKRange(double rho)
	    {
		m_gaik.setRange(rho);
	    }
	    
	    double getIKRange(void) const
	    {
		return m_gaik.getRange();
	    }
	    
	    virtual bool solve(double solveTime)
	    {
		SpaceInformationKinematic *si     = dynamic_cast<SpaceInformationKinematic*>(_P::m_si);
		GoalRegion                *goal_r = dynamic_cast<GoalRegion*>(si->getGoal());
		GoalState                 *goal_s = dynamic_cast<GoalState*>(si->getGoal());
		unsigned int                  dim = si->getStateDimension();
		
		if (goal_s)
		    return _P::solve(solveTime);
		
		if (!goal_r)
		{
		    _P::m_msg.error("IKPlanner: Unknown type of goal (or goal undefined)");
		    return false;
		}
		
		bool foundStart = false;
		for (unsigned int i = 0 ; i < _P::m_si->getStartStateCount() ; ++i)
		{
		    State *st = dynamic_cast<State*>(si->getStartState(i));
		    if (!st || !si->isValid(st))
			_P::m_msg.error("IKPlanner: Initial state is invalid!");
		    else
			foundStart = true;
		}    
		
		if (!foundStart)
		{
		    _P::m_msg.error("IKPlanner: Motion planning trees could not be initialized!");
		    return false;
		}
		
		time_utils::Time endTime = time_utils::Time::now() + time_utils::Duration(solveTime);
		
		/* memory for temporary goal */
		GoalState *stateGoal = new GoalState(si);
		stateGoal->state = new State(dim);
		stateGoal->threshold = goal_r->threshold;
		
		bool solved = false;
		unsigned int step = 0;
		
		while (!solved)
		{
		    step++;
		    double time_left = (endTime - time_utils::Time::now()).toSeconds();
		    if (time_left <= 0.0)
			break;
		    if (m_gaik.solve(time_left * 0.5, static_cast<State*>(stateGoal->state)))
		    {
			/* change goal to a state one */
			si->forgetGoal();
			si->setGoal(stateGoal);
			
			/* run _P on the new goal */
			_P::clear();
			time_left = (endTime - time_utils::Time::now()).toSeconds();
			_P::m_msg.inform("IKPlanner: Using GAIK goal state for the planner (step %u, %g seconds remaining)", step, time_left);
			solved = _P::solve(time_left);
			
			/* restore user-set goal */
			si->forgetGoal();
			si->setGoal(goal_r);
			
			/* copy solution to actual goal instance */
			if (solved)
			{
			    double dist = -1.0;
			    bool approx = !goal_r->isSatisfied(stateGoal->state, &dist);
			    if (approx)
				_P::m_msg.warn("IKPlanner: Found approximate solution");
			    goal_r->setSolutionPath(stateGoal->getSolutionPath(), approx);
			    goal_r->setDifference(dist);
			    stateGoal->forgetSolutionPath();
			}
			else
			    goal_r->setSolutionPath(NULL);
		    }
		}
		
		delete stateGoal;
		return solved;
	    }
	    
	protected:
	    
	    GAIK m_gaik;
	    
	};
	
    }
}

#endif
