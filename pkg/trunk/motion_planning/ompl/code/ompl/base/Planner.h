#ifndef OMPL_BASE_MOTION_PLANNER_
#define OMPL_BASE_MOTION_PLANNER_

#include "ompl/base/General.h"
#include "ompl/base/SpaceInformation.h"

namespace ompl
{

    ForwardClassDeclaration(Planner);
    
    class Planner
    {
	
    public:
	
	Planner(SpaceInformation_t si)
	{
	    m_si = si;
	}
	
	virtual ~Planner(void)
	{
	}
	
	virtual bool solve(double solveTime) = 0;
	virtual void clear(void) = 0;
	
    protected:
	
	SpaceInformation_t m_si;
	
    };    

}


#endif
