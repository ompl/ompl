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

#ifndef OMPL_EXTENSION_SAMPLINGBASED_KINEMATIC_SPACE_INFORMATION_KINEMATIC_
#define OMPL_EXTENSION_SAMPLINGBASED_KINEMATIC_SPACE_INFORMATION_KINEMATIC_

#include "ompl/base/SpaceInformation.h"
#include <random_utils/random_utils.h>

namespace ompl
{

    ForwardClassDeclaration(SpaceInformationKinematic);

    class SpaceInformationKinematic : public SpaceInformation
    {
    public:
	
	SpaceInformationKinematic() : SpaceInformation()
	{
	    random_utils::init(&m_rngState);

	    m_isValidStateFn     = NULL;
	    m_isValidStateFnData = NULL;
	    
	    m_smoother.rangeRatio    = 0.2;
	    m_smoother.maxSteps      = 10;
	    m_smoother.maxEmptySteps = 3;
	}
	
	virtual ~SpaceInformationKinematic(void)
	{
	}
	
	ForwardClassDeclaration(StateKinematic);
	
	class StateKinematic : public State
	{
	public:
	    
	    StateKinematic(void) : State()
	    {
		values = NULL;
	    }
	    
	    StateKinematic(const unsigned int dimension) : State()
	    {
		values = new double[dimension];
	    }
	    
	    virtual ~StateKinematic(void)
	    {
		if (values)
		    delete[] values;
	    }
	    
	    double *values;
	};

	ForwardClassDeclaration(GoalRegionKinematic);

	class GoalRegionKinematic : public Goal
	{
	public:
	    
	    GoalRegionKinematic(SpaceInformation_t si) : Goal(si)
	    {
		threshold = 0.0;
	    }
	    
	    virtual ~GoalRegionKinematic(void)
	    {
	    }
	    	  
	    virtual double distanceGoal(StateKinematic_t s) = 0;
	    	    
	    double threshold;
	};

	ForwardClassDeclaration(GoalStateKinematic);

	class GoalStateKinematic : public GoalRegionKinematic
	{
	public:
	    
	    GoalStateKinematic(SpaceInformation_t si) : GoalRegionKinematic(si)
	    {
		state = NULL;
	    }
	    
	    virtual ~GoalStateKinematic(void)
	    {
		if (state)
		    delete state;
	    }
	    
	    virtual double distanceGoal(StateKinematic_t s)
	    {
		return static_cast<SpaceInformationKinematic_t>(m_si)->distance(s, state);
	    }
	    
	    StateKinematic_t state;
	};
	
	ForwardClassDeclaration(PathKinematic);

	class PathKinematic : public Path
	{
	public:
	    
	    PathKinematic(SpaceInformation_t si) : Path(si)
	    {
	    }
	    
	    virtual ~PathKinematic(void)
	    {
		freeMemory();
	    }
	    	  
	    std::vector<StateKinematic_t> states;
	    
	protected:
	    
	    void freeMemory(void)
	    {
		for (unsigned int i = 0 ; i < states.size() ; ++i)
		    delete states[i];
	    }
	};
	
	struct StateComponent
	{
	    StateComponent(void)
	    {
		type = UNKNOWN;
	    }
	    
	    enum
		{ UNKNOWN, NORMAL, ANGLE, QUATERNION, FIXED }
		   type;
	    double minValue;
	    double maxValue;
	    double resolution;
	};

	typedef bool (*IsStateValidFn)(const StateKinematic_t, void*);
       
	void setStateValidFn(IsStateValidFn fun, void *data)
	{
	    m_isValidStateFn     = fun;
	    m_isValidStateFnData = data;
	}
	
	virtual void printState(const StateKinematic_t state, FILE* out = stdout) const;
	virtual void copyState(StateKinematic_t destination, const StateKinematic_t source)
	{
	    memcpy(destination->values, source->values, sizeof(double) * m_stateDimension);
	}
		
	unsigned int getStateDimension(void) const
	{
	    return m_stateDimension;
	}

	const StateComponent& getStateComponent(unsigned int index) const
	{
	    return m_stateComponent[index];
	}
	
	virtual double distance(const StateKinematic_t s1, const StateKinematic_t s2);

	virtual void sample(StateKinematic_t state);
	virtual void sampleNear(StateKinematic_t state, const StateKinematic_t near, double rho);	
	virtual void smoothVertices(PathKinematic_t path);	
	virtual bool checkMotion(const StateKinematic_t s1, const StateKinematic_t s2);
	virtual bool isValid(const StateKinematic_t state)
	{
	    return m_isValidStateFn(state, m_isValidStateFnData);
	}
	
	virtual void printSettings(FILE *out = stdout) const;
	
    protected:
		
	unsigned int                m_stateDimension;
	std::vector<StateComponent> m_stateComponent;
	IsStateValidFn              m_isValidStateFn;
	void                       *m_isValidStateFnData;
	
	struct
	{
	    double       rangeRatio;
	    unsigned int maxSteps;
	    unsigned int maxEmptySteps;
	    
	} m_smoother;

	random_utils::rngState      m_rngState;
	
    };
    
}

#endif
