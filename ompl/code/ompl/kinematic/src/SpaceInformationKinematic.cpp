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

#include "ompl/kinematic/SpaceInformationKinematic.h"

#include <ros/console.h>
#include <angles/angles.h>
#include <algorithm>

void ompl::kinematic::SpaceInformationKinematic::setup(void)
{
    if (!m_stateDistanceEvaluator)
	ROS_FATAL("No state distance evaluator defined");
    
    if (!m_stateValidityChecker)
	ROS_FATAL("No state validity checker defined");
    
    if (!m_stateInterpolator)
	ROS_FATAL("No state interpolator defined");

    SpaceInformation::setup();
}

bool ompl::kinematic::SpaceInformationKinematic::checkPath(const PathKinematic *path) const
{
    bool result = path != NULL;
    if (result && path->states.size() > 0)
    {
	if (isValid(path->states[0]))
	{
	    int last = path->states.size() - 1;
	    for (int j = 0 ; result && j < last ; ++j)
		if (!checkMotion(path->states[j], path->states[j + 1]))
		    result = false;
	}
	else
	    result = false;
    }
    return result;
}

void ompl::kinematic::SpaceInformationKinematic::interpolatePath(PathKinematic *path, double factor) const
{
    std::vector<base::State*> newStates;
    const int n1 = path->states.size() - 1;
    
    for (int i = 0 ; i < n1 ; ++i)
    {
	base::State *s1 = path->states[i];
	base::State *s2 = path->states[i + 1];
	
	newStates.push_back(s1);
	
	std::vector<base::State*> block;
	m_stateInterpolator->getStates(s1, s2, block, factor, false, true);
	newStates.insert(newStates.end(), block.begin(), block.end());
    }
    newStates.push_back(path->states[n1]);
    
    path->states.swap(newStates);
}

