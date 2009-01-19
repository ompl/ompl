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

#ifndef OMPL_EXTENSION_SAMPLINGBASED_KINEMATIC_SPACE_INFORMATION_KINEMATIC_
#define OMPL_EXTENSION_SAMPLINGBASED_KINEMATIC_SPACE_INFORMATION_KINEMATIC_

#include "ompl/base/SpaceInformation.h"

#include <random_utils/random_utils.h>
#include <cassert>

/** Main namespace */
namespace ompl
{
    
    static const int PLAN_TO_GOAL_STATE  = 1;
    static const int PLAN_TO_GOAL_REGION = 2;

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
	    random_utils::init(&m_rngState);
	    m_stateDistanceEvaluator = &m_defaultDistanceEvaluator;	    
	}
	
	/** Destructor */
	~SpaceInformationKinematic(void)
	{
	}

	/** Forward class declaration */
	ForwardClassDeclaration(StateKinematic);

	/** Forward class declaration */
	ForwardClassDeclaration(GoalRegionKinematic);

	/** Forward class declaration */
	ForwardClassDeclaration(GoalStateKinematic);

	/** Forward class declaration */
	ForwardClassDeclaration(PathKinematic);

	/** Definition of a kinematic state: an array of doubles */
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

	/** Definition of a goal region */
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

	    /** Decide whether a given state is part of the goal
		region. Returns true if the distance to goal is less
		than the threshold */
	    virtual bool isSatisfied(State_t s, double *distance = NULL);

	    /** Compute the distance to the goal (heuristic) */
	    virtual double distanceGoal(StateKinematic_t s) = 0;

	    /** Print information about the goal data structure to the
		screen */
	    virtual void print(std::ostream &out = std::cout) const;
	    
	    /** The maximum distance that is allowed to the goal */
	    double threshold;
	};

	/** Definition of a goal state */
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

	    /** Compute the distance to the goal (heuristic) */
	    virtual double distanceGoal(StateKinematic_t s);	    

	    /** Print information about the goal data structure to the
		screen */
	    virtual void print(std::ostream &out = std::cout) const;
	    
	    /** The goal state */
	    StateKinematic_t state;
	};
	
	/** Definition of a kinematic path */
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
	    
	    /** The list of states that make up the path */
	    std::vector<StateKinematic_t> states;
	    
	protected:
	    
	    void freeMemory(void);
	    
	};

	/** Definition of a distance evaluator: the square of the L2 norm */
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
		{ UNKNOWN, NORMAL, WRAPPING_ANGLE, QUATERNION }
		   type;
	    double minValue;
	    double maxValue;
	    double resolution;
	};
       	
	/** Print a state to a stream */
	virtual void printState(const StateKinematic_t state, std::ostream &out = std::cout) const;

	/** Copy a state to another */
	virtual void copyState(StateKinematic_t destination, const StateKinematic_t source);
	
	/** Return the dimension of the state space */
	unsigned int getStateDimension(void) const;
	
	/** Get information about a component of the state space */
	const StateComponent& getStateComponent(unsigned int index) const;
	
	/** Compute the distance between two states */
	double distance(const StateKinematic_t s1, const StateKinematic_t s2);
	
	/** Sample a state */
	virtual void sample(StateKinematic_t state);

	/** Sample a state near another, within given bounds */
	virtual void sampleNear(StateKinematic_t state, const StateKinematic_t near, const double rho);

	/** Sample a state near another, within given bounds */
	virtual void sampleNear(StateKinematic_t state, const StateKinematic_t near, const std::vector<double> &rho);
	
	/** Check if the path between two motions is valid using subdivision */
	virtual bool checkMotionSubdivision(const StateKinematic_t s1, const StateKinematic_t s2);

	/** Incrementally check if the path between two motions is valid */
	virtual bool checkMotionIncremental(const StateKinematic_t s1, const StateKinematic_t s2);
	
	/** Insert states in a path, at the collision checking resolution */
	virtual void interpolatePath(PathKinematic_t path, double factor = 1.0);

	/** Check if a given state is valid or not */
	bool isValid(const StateKinematic_t state);
	
	/** Print information about the current instance of the state space */
	virtual void printSettings(std::ostream &out = std::cout) const;
	
	/** Perform additional tasks to finish the initialization of
	    the space information */
	virtual void setup(void);
	
    protected:
		
	unsigned int                            m_stateDimension;
	std::vector<StateComponent>             m_stateComponent;
	StateKinematicL2SquareDistanceEvaluator m_defaultDistanceEvaluator;
	
	random_utils::rngState                  m_rngState;
	
    };
    
}

#endif
