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

#ifndef OMPL_BASE_SPACE_INFORMATION_
#define OMPL_BASE_SPACE_INFORMATION_

#include "ompl/base/General.h"
#include <vector>
#include <cassert>

/** Main namespace */
namespace ompl
{

    /** Forward class declaration */
    ForwardClassDeclaration(SpaceInformation);
    
    /** The base class for space information. This contains all the
	information about the space planning is done in */
    class SpaceInformation
    {
    public:
	
	/** Constructor */
	SpaceInformation(void)
	{
	    m_goal = NULL;
	    m_setup = false;
	    m_stateDistanceEvaluator = NULL;
	    m_stateValidityChecker = NULL;
	}
	
	/** Destructor */
	virtual ~SpaceInformation(void)
	{
	}
	
	/************************************************************/
	/* States                                                   */
	/************************************************************/

	/** Forward class declaration */
	ForwardClassDeclaration(State);
	
	/** Abstract definition of a state */
	class State
	{
	public:
	    virtual ~State(void)
	    {
	    }
	};

	/** Forward class declaration */
	ForwardClassDeclaration(StateValidityChecker);	
	
	/** Abstract definition for a class checking the validity of states. The () operator must be defined. */
	class StateValidityChecker
	{
	public:
	    /** Destructor */
	    virtual ~StateValidityChecker(void)
	    {
	    }
	    
	    /** Return true if the state is valid */
	    virtual bool operator()(const State_t state) = 0;
	};
	
	/** Forward class declaration */
	ForwardClassDeclaration(StateDistanceEvaluator);
	
	/** Abstract definition for a class evaluating distance between states. The () operator must be defined. */
	class StateDistanceEvaluator
	{
	public:
	    /** Destructor */
	    virtual ~StateDistanceEvaluator(void)
	    {
	    }
	    /** Return true if the state is valid */
	    virtual double operator()(const State_t state1, const State_t state2) = 0;
	};
	
	/** Add a start state */
	void addStartState(State_t state)
	{
	    m_startStates.push_back(state);
	}
	
	/** Clear all start states (memory is freed) */
	void clearStartStates(void)
	{
	    for (unsigned int i = 0 ; i < m_startStates.size() ; ++i)
		delete m_startStates[i];
	    m_startStates.clear();
	}
	
	/** Clear all start states but do not free memory */
	void forgetStartStates(void)
	{
	    m_startStates.clear();
	}
	
	/** Returns the number of start states */
	unsigned int getStartStateCount(void) const
	{
	    return m_startStates.size();
	}
	
	/** Returns a specific start state */
	State_t getStartState(unsigned int index) const
	{
	    return m_startStates[index];
	}
	
	/** Set the instance of the distance evaluator to use. This is
	    only needed by some planning algorithms. No memory freeing is performed. */
	void setStateDistanceEvaluator(StateDistanceEvaluator_t sde)
	{
	    m_stateDistanceEvaluator = sde;
	}

	/** Return the instance of the used state distance evaluator */
	StateDistanceEvaluator_t getStateDistanceEvaluator(void) const
	{
	    return m_stateDistanceEvaluator;
	}	
	
	/** Set the instance of the validity checker to use. No memory freeing is performed. */
	void setStateValidityChecker(StateValidityChecker_t svc)
	{
	    m_stateValidityChecker = svc;
	}
	
	/** Return the instance of the used state validity checker */
	StateValidityChecker_t getStateValidityChecker(void) const
	{
	    return m_stateValidityChecker;
	}
	
	/************************************************************/
	/* Paths                                                    */
	/************************************************************/

	/** Forward declaration */
	ForwardClassDeclaration(Path);
	
	/** Abstract definition of a path */
	class Path
	{
	public:
	    
	    /** Constructor. A path must always know the space information it is part of */
	    Path(SpaceInformation_t si)
	    {
		m_si = si;
	    }

	    /** Destructor */
	    virtual ~Path(void)
	    {
	    }

	    /** Returns the space information this path is part of */
	    SpaceInformation_t getSpaceInformation(void) const
	    {
		return m_si;
	    }
	    
	protected:
	    
	    SpaceInformation_t m_si;
	};
	
	    

	/************************************************************/
	/* Goals                                                    */
	/************************************************************/
	
	/** Forward declaration */
	ForwardClassDeclaration(Goal);
	
	/** Abstract definition of goals. Will contain solutions, if found */
	class Goal
	{
	public:
	    
	    /** Constructor. The goal must always know the space information it is part of */
	    Goal(SpaceInformation_t si)
	    {
		m_si   = si;
		m_path = NULL;
		m_difference = -1.0;
		m_approximate = false;
	    }
	    
	    /** Destructor. Clears the solution as well */
	    virtual ~Goal(void)
	    {
		if (m_path)
		    delete m_path;
	    }

	    /** Return true if the state statisfies the goal
	     *  constraints.  If the state does not satisfy the
	     *  constraints, set the distance of how far the state
	     *  is from the goal. */
	    virtual bool isSatisfied(State_t s, double *distance) = 0;
	    
	    /** Returns the space information this goal is part of */
	    SpaceInformation_t getSpaceInformation(void) const
	    {
		return m_si;
	    }
	    
	    /** Returns true if a solution path has been found */
	    bool isAchieved(void) const
	    {
		return m_path != NULL;
	    }

	    /** Return the found solution path */
	    Path_t getSolutionPath(void) const
	    {
		return m_path;
	    }
	    
	    /** Forget the solution path. Memory is not freed. This is
		useful when the user wants to keep the solution path
		but wants to clear the goal. The user takes
		responsibilty to free the memory for the solution
		path. */
	    void forgetSolutionPath(void)
	    {
		m_path = NULL;
	    }	    

	    /** Update the solution path. If a previous solution path exists, it is deleted. */
	    void setSolutionPath(Path_t path, bool approximate = false)
	    {
		if (m_path)
		    delete m_path;
		m_path = path;
		m_approximate = approximate;
	    }
	    
	    /** If a difference between the desired solution and the
	     solution found is computed by the planner, this functions
	     returns it */
	    double getDifference(void) const
	    {
		return m_difference;
	    }
	    
	    /** Set the difference between the found solution path and
		the desired solution path */
	    void setDifference(double difference)
	    {
		m_difference = difference;
	    }
	    
	    /** Return true if the found solution is approximate */
	    bool isApproximate(void) const
	    {
		return m_approximate;
	    }
	    
	    /** Print information about the goal */
	    virtual void print(std::ostream &out = std::cout) const
	    {
		out << "Goal memory address " << reinterpret_cast<const void*>(this) << std::endl;
	    }
	    
	protected:
	    
	    /** solution path, if found */
	    Path_t             m_path;
	    
	    /** the space information for this goal */
	    SpaceInformation_t m_si;
	    
	    /** the achieved difference between the found solution and the desired goal */
	    double             m_difference;

	    /** true if goal was not achieved, but an approximate solution was found */
	    bool               m_approximate;
	    
	};
	

	/** Set the goal. The memory for a previous goal is freed. */
	void setGoal(Goal_t goal)
	{
	    if (m_goal)
		delete m_goal;
	    m_goal = goal;
	}
	
	/** Clear the goal. Memory is freed. */
	void clearGoal(void)
	{
	    if (m_goal)
		delete m_goal;
	    m_goal = NULL;
	}

	/** Return the current goal */
	Goal_t getGoal(void) const
	{
	    return m_goal;
	}
	
	/** Clear the goal, but do not free its memory */
	void forgetGoal(void)
	{
	    m_goal = NULL;
	}
	
	/************************************************************/
	/* Utility functions                                        */
	/************************************************************/
	
	/** Perform additional setup tasks (run once, before use) */
	virtual void setup(void)
	{
	    assert(m_stateValidityChecker);
	    m_setup = true;
	}
	
	/** Check if a given state is valid or not */
	bool isValid(const State_t state)
	{
	    return (*m_stateValidityChecker)(state);
	}
	
    protected:

	bool                     m_setup;
	msg::Interface           m_msg;

	std::vector<State_t>     m_startStates;
	Goal_t                   m_goal;
	StateValidityChecker_t   m_stateValidityChecker;
	StateDistanceEvaluator_t m_stateDistanceEvaluator;
	
    };
    
	

}

#endif
