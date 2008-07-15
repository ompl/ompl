#ifndef OMPL_EXTENSION_SAMPLINGBASED_KINEMATIC_EXTENSION_RRT_
#define OMPL_EXTENSION_SAMPLINGBASED_KINEMATIC_EXTENSION_RRT_

#include "ompl/base/MotionPlanner.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/extension/samplingbased/kinematic/SpaceInformationKinematic.h"

namespace ompl
{

    class RRT : public MotionPlanner
    {
    public:

        RRT(SpaceInformation_t si) : MotionPlanner(si)
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
       
	void freeMemory(void)
	{
	    std::vector<SpaceInformationKinematic::MotionKinematic_t> motions;
	    m_nn.list(motions);
	    for (unsigned int i = 0 ; i < motions.size() ; ++i)
		delete motions[i];
	}
		
	struct distanceFunction
	{
	    double operator()(const SpaceInformationKinematic::MotionKinematic_t a,
			      const SpaceInformationKinematic::MotionKinematic_t b,
			      void *data)
	    {
		return reinterpret_cast<SpaceInformationKinematic_t>(data)->distance(a->state, b->state);
	    }
	};
	
	NearestNeighbors<SpaceInformationKinematic::MotionKinematic_t, distanceFunction> m_nn;
	double                                                                           m_goalBias;
	double                                                                           m_rho;	
	random_utils::rngState                                                           m_rngState;	
    };

}

#endif
