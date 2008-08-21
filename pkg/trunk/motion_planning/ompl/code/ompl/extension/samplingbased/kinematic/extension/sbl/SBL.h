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

/** \Author Ioan Sucan */

#ifndef OMPL_EXTENSION_SAMPLINGBASED_KINEMATIC_EXTENSION_SBL_
#define OMPL_EXTENSION_SAMPLINGBASED_KINEMATIC_EXTENSION_SBL_

#include "ompl/base/Planner.h"
#include "ompl/datastructures/Grid.h"
#include "ompl/extension/samplingbased/kinematic/SpaceInformationKinematic.h"

namespace ompl
{

    ForwardClassDeclaration(SBL);
    
    class SBL : public Planner
    {
    public:

        SBL(SpaceInformation_t si) : Planner(si)
	{
	    random_utils::random_init(&m_rngState);
	    m_rho = 0.1;	    
	}

	virtual ~SBL(void)
	{
	    freeMemory();
	}
	
	virtual bool solve(double solveTime);
	
	virtual void clear(void)
	{
	    freeMemory();
	}
	
    protected:

       	ForwardClassDeclaration(Motion);
	
	class Motion
	{
	public:
	    
	    Motion(void)
	    {
		parent = NULL;
		state  = NULL;
		valid  = false;
	    }
	    
	    Motion(unsigned int dimension)
	    {
		state  = new SpaceInformationKinematic::StateKinematic(dimension);
		parent = NULL;
		valid  = false;
	    }
	    
	    virtual ~Motion(void)
	    {
		if (state)
		    delete state;
	    }
	    
	    SpaceInformationKinematic::StateKinematic_t state;
	    Motion_t                                    parent;
	    bool                                        valid;
	    std::vector<Motion_t>                       children;
	};

	void freeMemory(void)
	{
	}
	
	Motion_t selectMotion(Grid<Motion_t> &grid);	
	void removeMotion(Grid<Motion_t> &grid, Motion_t motion);
	void addMotion(Grid<Motion_t> &grid, Motion_t motion);

	Grid<Motion_t>         m_gStart;
	Grid<Motion_t>         m_gGoal;
	
	double                 m_rho;	
	random_utils::rngState m_rngState;	
    };

}

#endif
