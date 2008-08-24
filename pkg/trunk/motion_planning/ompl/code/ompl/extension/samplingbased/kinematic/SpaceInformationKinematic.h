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

#ifndef OMPL_EXTENSION_SAMPLINGBASED_KINEMATIC_SPACE_INFORMATION_KINEMATIC_
#define OMPL_EXTENSION_SAMPLINGBASED_KINEMATIC_SPACE_INFORMATION_KINEMATIC_

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/util/random.h"

namespace ompl
{
    
    /** Forward class declaration */
    ForwardClassDeclaration(SpaceInformationKinematic);
    
    /** Space information useful for kinematic planning */
    class SpaceInformationKinematic : public SpaceInformation
    {
    public:
	
	/** Constructor; setup() needs to be called as well, before use */
        SpaceInformationKinematic(void) : SpaceInformation(),
	                                  m_defaultDistanceEvaluator(this)
	{
	    random_utils::random_init(&m_rngState);
	    m_stateDistanceEvaluator = &m_defaultDistanceEvaluator;	    
	}
	
	/** Destructor */
	~SpaceInformationKinematic(void)
	{
	}

	ForwardClassDeclaration(StateKinematic);
	ForwardClassDeclaration(GoalRegionKinematic);
	ForwardClassDeclaration(GoalStateKinematic);
	ForwardClassDeclaration(PathKinematic);

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

	    virtual bool isSatisfied(State_t s, double *distance)
	    {
		double d2g = distanceGoal(static_cast<StateKinematic_t>(s));
		if (distance)
		    *distance = d2g;
		return d2g < threshold;
	    }
	    
	    virtual double distanceGoal(StateKinematic_t s) = 0;
	    
	    virtual void print(std::ostream &out = std::cout) const
	    {
		out << "Goal region, threshold = " << threshold << ", memory address = " << reinterpret_cast<const void*>(this) << std::endl;
	    }
	    
	    double threshold;
	};


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
	    
	    virtual void print(std::ostream &out = std::cout) const
	    {
		out << "Goal state, threshold = " << threshold << ", memory address = " << reinterpret_cast<const void*>(this) << ", state = ";
		static_cast<SpaceInformationKinematic_t>(m_si)->printState(state, out);
	    }
	    
	    StateKinematic_t state;
	};
	

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
	
	class StateKinematicL2SquareDistanceEvaluator : public StateDistanceEvaluator
	{
	public:
	    StateKinematicL2SquareDistanceEvaluator(SpaceInformationKinematic_t si) : StateDistanceEvaluator()
	    {
		m_si = si;
	    }
	    
	    virtual double operator()(const State_t state1, const State_t state2);
	    
	protected:
	    
	    SpaceInformationKinematic_t m_si;	    
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
       	
	virtual void printState(const StateKinematic_t state, std::ostream &out = std::cout) const;
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
	
	double distance(const StateKinematic_t s1, const StateKinematic_t s2)
	{
	    return (*m_stateDistanceEvaluator)(static_cast<const State_t>(s1), static_cast<const State_t>(s2));
	}
	
	virtual void sample(StateKinematic_t state);
	virtual void sampleNear(StateKinematic_t state, const StateKinematic_t near, double rho);

	virtual bool checkMotionSubdivision(const StateKinematic_t s1, const StateKinematic_t s2);
	virtual bool checkMotionIncremental(const StateKinematic_t s1, const StateKinematic_t s2);
	virtual void interpolatePath(PathKinematic_t path);

	bool isValid(const StateKinematic_t state)
	{
	    return (*m_stateValidityChecker)(static_cast<const State_t>(state));
	}
	
	virtual void printSettings(std::ostream &out = std::cout) const;
	
	virtual void setup(void)
	{
	    assert(m_stateDimension > 0);
	    assert(m_stateComponent.size() == m_stateDimension);
	    assert(m_stateValidityChecker);
	    assert(m_stateDistanceEvaluator);
	    SpaceInformation::setup();
	}
	
    protected:
		
	unsigned int                            m_stateDimension;
	std::vector<StateComponent>             m_stateComponent;
	StateKinematicL2SquareDistanceEvaluator m_defaultDistanceEvaluator;
	
	random_utils::rngState                  m_rngState;
	
    };
    
}

#endif
