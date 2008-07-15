#ifndef OMPL_EXTENSION_SAMPLINGBASED_KINEMATIC_SPACE_INFORMATION_KINEMATIC_
#define OMPL_EXTENSION_SAMPLINGBASED_KINEMATIC_SPACE_INFORMATION_KINEMATIC_

#include "ompl/base/SpaceInformation.h"

namespace ompl
{

    ForwardClassDeclaration(SpaceInformationKinematic);

    class SpaceInformationKinematic : public SpaceInformation
    {
    public:
	
	SpaceInformationKinematic(void) : SpaceInformation()
	{
	    random_utils::init(&m_rngState);
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
	
	
	ForwardClassDeclaration(MotionKinematic);
	
	class MotionKinematic 
	{
	public:
	    
	    MotionKinematic(void)
	    {
		parent = NULL;
		state  = NULL;
		valid  = false;
	    }
	    
	    MotionKinematic(unsigned int dimension)
	    {
		state  = new StateKinematic(dimension);
		parent = NULL;
		valid  = false;
	    }
	    
	    virtual ~MotionKinematic(void)
	    {
		if (state)
		    delete state;
	    }
	    
	    StateKinematic_t  state;
	    MotionKinematic_t parent;
	    bool              valid;
	    
	};
	
	void printState(StateKinematic_const_t state, FILE* out = stdout)
	{
	    for (int i = 0 ; i < m_stateDimension ; ++i)
		fprintf(out, "%0.6f ", state->values[i]);
	    fprintf(out, "\n");
	}
	
	void copyState(StateKinematic_t destination, StateKinematic_const_t source)
	{
	    memcpy(destination->values, source->values, sizeof(double) * m_stateDimension);
	}
		
	unsigned int getStateDimension(void) const
	{
	    return m_stateDimension;
	}

	virtual double distance(StateKinematic_t s1, StateKinematic_t s2);

	virtual void sample(StateKinematic_t state);
	virtual void sampleNear(StateKinematic_t state, StateKinematic_t near, double rho);	

    protected:

	struct StateComponent
	{
	    enum
		{ NORMAL, FIXED	}
		type;	    
	    double minValue;
	    double maxValue;
	};
		
	unsigned int                m_stateDimension;
	std::vector<StateComponent> m_stateComponent;
	random_utils::rngState      m_rngState;
	
    };
    
}

#endif
