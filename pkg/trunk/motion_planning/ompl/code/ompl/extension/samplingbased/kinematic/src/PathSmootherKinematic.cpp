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

#include "ompl/extension/samplingbased/kinematic/PathSmootherKinematic.h"
#include <cstdlib>

void ompl::sb::PathSmootherKinematic::smoothVertices(PathKinematic *path)
{
    if (!path || path->states.size() < 3)
	return;    
    
    unsigned int nochange = 0;
    
    for (unsigned int i = 0 ; i < m_maxSteps && nochange < m_maxEmptySteps ; ++i, ++nochange)
    {
	int count = path->states.size();
	int maxN  = count - 1;
	int range = 1 + (int)((double)count * m_rangeRatio);
	
	int p1 = m_rng.uniformInt(0, maxN);
	int p2 = m_rng.uniformInt(std::max(p1 - range, 0), std::min(maxN, p1 + range));
	if (abs(p1 - p2) < 2)
	{
	    if (p1 < maxN - 1)
		p2 = p1 + 2;
	    else
		if (p1 > 1)
		    p2 = p1 - 2;
		else
		    continue;
	}

	if (p1 > p2)
	    std::swap(p1, p2);
	
	if (m_si->checkMotionSubdivision(path->states[p1], path->states[p2]))
	{
	    for (int i = p1 + 1 ; i < p2 ; ++i)
		delete path->states[i];
	    path->states.erase(path->states.begin() + p1 + 1, path->states.begin() + p2);
	    nochange = 0;
	}
    }
}

void ompl::sb::PathSmootherKinematic::smoothMax(PathKinematic *path)
{
    smoothVertices(path);
    m_si->interpolatePath(path, 3.0);
    smoothVertices(path);
    removeRedundantCommands(path);    
}

void ompl::sb::PathSmootherKinematic::removeRedundantCommands(PathKinematic *path) const
{
    if (!path || path->states.size() < 3)
	return;

    unsigned int dim = m_si->getStateDimension();
    int         last = path->states.size() - 1;
    double   *backup = new double[path->states.size()];
    
    for (unsigned int i = 0 ; i < dim ; ++i)
    {
	double command = path->states[0]->values[i];
	if (command != path->states[last]->values[i])
	    continue;
	
	bool diff = false;
	for (int j = 1 ; j < last ; ++j)
	{
	    backup[j] = path->states[j]->values[i];
	    if (path->states[j]->values[i] != command)
	    {
		path->states[j]->values[i] = command;
		diff = true;
	    }
	}
	
	if (diff)
	{
	    bool change_back = false;	
	    for (int j = 0 ; change_back && j < last ; ++j)
		if (!m_si->checkMotionSubdivision(path->states[j], path->states[j + 1]))
		    change_back = true;
	    if (change_back)
	    {
		for (int j = 1 ; j < last ; ++j)
		    path->states[j]->values[i] = backup[j];
	    }
	}
    }
    
    delete[] backup;
}
