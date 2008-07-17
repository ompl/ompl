#ifndef OMPL_DATASTRUCTURES_NEAREST_NEIGHBORS_
#define OMPL_DATASTRUCTURES_NEAREST_NEIGHBORS_

#include <vector>

namespace ompl
{

    template<typename _T, typename _DistanceFunction>
    class NearestNeighbors
    {
    public:
	NearestNeighbors(void)
	{
	    m_parameter = NULL;
	}
	
	virtual ~NearestNeighbors(void)
	{
	}

	void setDataParameter(void *parameter)
	{
	    m_parameter = parameter;
	}
	
	virtual void clear(void) = 0;
	virtual void add(_T &data) = 0;
	virtual _T nearest(_T &data) const = 0;
	virtual unsigned int size(void) const = 0;		
	virtual void list(std::vector<_T> &data) const = 0;
		
    protected:
	
	void            *m_parameter;
	
    };
    
    
}

#endif
