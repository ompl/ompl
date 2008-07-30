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

#include "ompl/extension/samplingbased/kinematic/extension/sbl/SBL.h"

bool ompl::SBL::solve(double solveTime)
{
    SpaceInformationKinematic_t                       si = dynamic_cast<SpaceInformationKinematic_t>(m_si); 
    SpaceInformationKinematic::GoalStateKinematic_t goal = dynamic_cast<SpaceInformationKinematic::GoalStateKinematic_t>(si->getGoal());
    unsigned int                                     dim = si->getStateDimension();
    
    if (!goal)
    {
	fprintf(stderr, "Unknown type of goal\n");
	return false;
    }
    
    time_utils::Time endTime = time_utils::Time::now() + time_utils::Duration(solveTime);

    if (m_gStart.size() == 0)
    {
	for (unsigned int i = 0 ; i < m_si->getStartStateCount() ; ++i)
	{
	    Motion_t motion = new Motion(dim);
	    si->copyState(motion->state, dynamic_cast<SpaceInformationKinematic::StateKinematic_t>(si->getStartState(i)));
	    if (si->isValid(motion->state))
	    {
		motion->valid = true;
		addMotion(m_gStart, motion);
	    }
	    else
	    {
		fprintf(stderr, "Initial state is in collision!\n");
		delete motion;
	    }	
	}
    }
    
    if (m_gGoal.size() == 0)
    {	   
	Motion_t motion = new Motion(dim);
	si->copyState(motion->state, goal->state);
	if (si->isValid(motion->state))
	{
	    motion->valid = true;
	    addMotion(m_gGoal, motion);
	}
	else
	{
	    fprintf(stderr, "Goal state is in collision!\n");
	    delete motion;
	}
    }
    
    if (m_gStart.size() == 0 || m_gGoal.size() == 0)
    {
	fprintf(stderr, "Motion planning trees could not be initialized!\n");
	return false;
    }
    
    Motion_t                                    solution  = NULL;
    SpaceInformationKinematic::StateKinematic_t xstate    = new SpaceInformationKinematic::StateKinematic(dim);
    bool                                        startGrid = true;
    
    while (time_utils::Time::now() < endTime)
    {
	Grid<Motion_t> &grid      = startGrid ? m_gStart : m_gGoal;
	startGrid = !startGrid;
	Grid<Motion_t> &otherGrid = startGrid ? m_gStart : m_gGoal;
	
	Motion_t existing = selectMotion(grid);
	si->sampleNear(xstate, existing->state, m_rho);
	
	/* create a motion */
	Motion_t motion = new Motion(dim);
	si->copyState(motion->state, xstate);
	motion->parent = existing;
	existing->children.push_back(motion);

	addMotion(grid, motion);
	
    }
   


    delete xstate;
	
    return goal->isAchieved();
}

ompl::SBL::Motion_t ompl::SBL::selectMotion(Grid<Motion_t> &grid)
{
    return NULL;
}

void ompl::SBL::removeMotion(Grid<Motion_t> &grid, Motion_t motion)
{
}

void ompl::SBL::addMotion(Grid<Motion_t> &grid, Motion_t motion)
{
}

