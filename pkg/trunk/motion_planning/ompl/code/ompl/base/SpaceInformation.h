#ifndef OMPL_BASE_SPACE_INFORMATION_
#define OMPL_BASE_SPACE_INFORMATION_

#include "ompl/base/General.h"
#include <vector>

namespace ompl
{

    ForwardClassDeclaration(SpaceInformation);
    
    class SpaceInformation
    {
    public:
	SpaceInformation(void)
	{
	}
	
	virtual ~SpaceInformation(void)
	{
	}
	

	/************************************************************/
	/* Paths                                                    */
	/************************************************************/

	ForwardClassDeclaration(Path);
	
	class Path
	{
	public:
	    
	    Path(SpaceInformation_t si)
	    {
		m_si = si;
	    }
	    
	    virtual ~Path(void)
	    {
	    }

	protected:
	    
	    SpaceInformation_t m_si;
	};
	
	    

	/************************************************************/
	/* Goals                                                    */
	/************************************************************/
	
	ForwardClassDeclaration(Goal);
	
	class Goal
	{
	public:

	    Goal(SpaceInformation_t si)
	    {
		m_si   = si;
		m_path = NULL;
	    }
	    
	    virtual ~Goal(void)
	    {
		if (m_path)
		    delete m_path;
	    }
	    
	    bool isAchieved(void) const
	    {
		return m_path != NULL;
	    }

	    Path_t getSolutionPath(void) const
	    {
		return m_path;
	    }
	    
	    void setSolutionPath(Path_t path)
	    {
		m_path = path;
	    }
	    
	protected:
	    
	    Path_t             m_path;
	    SpaceInformation_t m_si;
	};
	
	void addGoal(Goal_t goal)
	{
	    m_goals.push_back(goal);
	}
	
	void clearGoals(bool free = true)
	{
	    if (free)
		for (unsigned int i = 0 ; i < m_goals.size() ; ++i)
		    delete m_goals[i];
	    m_goals.clear();
	}

	unsigned int getGoalCount(void) const
	{
	    return m_goals.size();
	}
	
	Goal_t getGoal(unsigned int index) const
	{
	    return m_goals[index];
	}
	
	
	/************************************************************/
	/* States                                                   */
	/************************************************************/

	ForwardClassDeclaration(State);
	
	class State
	{
	public:
	    virtual ~State(void)
	    {
	    }
	};
	
	void addInitialState(State_t state)
	{
	    m_startStates.push_back(state);
	}
	
	void clearInitialStates(bool free = true)
	{
	    if (free)
		for (unsigned int i = 0 ; i < m_startStates.size() ; ++i)
		    delete m_startStates[i];
	    m_startStates.clear();
	}
	
	unsigned int getStartStateCount(void) const
	{
	    return m_startStates.size();
	}

	State_t getStartState(unsigned int index) const
	{
	    return m_startStates[index];
	}
	
    protected:
	
	std::vector<Goal_t>  m_goals;
	std::vector<State_t> m_startStates;
	
    };
    
	

}

#endif
