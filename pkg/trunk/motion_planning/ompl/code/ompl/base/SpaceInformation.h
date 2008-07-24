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
	    m_goal = NULL;	    
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
	
	void setGoal(Goal_t goal)
	{
	    if (m_goal)
		delete m_goal;
	    m_goal = goal;
	}
	
	void clearGoal(bool free = true)
	{
	    if (free && m_goal)
		delete m_goal;
	    m_goal = NULL;
	}

	Goal_t getGoal(void) const
	{
	    return m_goal;
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
	
	void addStartState(State_t state)
	{
	    m_startStates.push_back(state);
	}
	
	void clearStartStates(bool free = true)
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
	
	Goal_t               m_goal;
	std::vector<State_t> m_startStates;
	
    };
    
	

}

#endif
