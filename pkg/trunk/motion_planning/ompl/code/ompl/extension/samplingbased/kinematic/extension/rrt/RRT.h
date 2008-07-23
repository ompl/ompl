#ifndef OMPL_EXTENSION_SAMPLINGBASED_KINEMATIC_EXTENSION_RRT_
#define OMPL_EXTENSION_SAMPLINGBASED_KINEMATIC_EXTENSION_RRT_

#include "ompl/base/Planner.h"
#include "ompl/datastructures/NearestNeighborsLinear.h"
#include "ompl/extension/samplingbased/kinematic/SpaceInformationKinematic.h"

namespace ompl
{

    ForwardClassDeclaration(RRT);
    
    class RRT : public Planner
    {
    public:

        RRT(SpaceInformation_t si) : Planner(si)
	{
	    m_nn.setDataParameter(reinterpret_cast<void*>(dynamic_cast<SpaceInformationKinematic_t>(m_si)));
	    random_utils::init(&m_rngState);
	    m_goalBias = 0.05;	    
	    m_rho = 0.1;	    
	}

	virtual ~RRT(void)
	{
	    freeMemory();
	}
	
	virtual bool solve(double solveTime);
	
	virtual void clear(void)
	{
	    freeMemory();
	    m_nn.clear();
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
	    }
	    
	    Motion(unsigned int dimension)
	    {
		state  = new SpaceInformationKinematic::StateKinematic(dimension);
		parent = NULL;
	    }
	    
	    virtual ~Motion(void)
	    {
		if (state)
		    delete state;
	    }
	    
	    SpaceInformationKinematic::StateKinematic_t state;
	    Motion_t                                    parent;
	    
	};

	void freeMemory(void)
	{
	    std::vector<Motion_t> motions;
	    m_nn.list(motions);
	    for (unsigned int i = 0 ; i < motions.size() ; ++i)
		delete motions[i];
	}
	
	struct distanceFunction
	{
	    double operator()(const Motion_t a, const Motion_t b, void *data)
	    {
		return reinterpret_cast<SpaceInformationKinematic_t>(data)->distance(a->state, b->state);
	    }
	};
	
	NearestNeighborsLinear<Motion_t, distanceFunction> m_nn;
	double                                       m_goalBias;
	double                                       m_rho;	
	random_utils::rngState                       m_rngState;	
    };

}

#endif
