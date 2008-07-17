#ifndef OMPL_DATASTRUCTURES_NEAREST_NEIGHBORS_LINEAR_
#define OMPL_DATASTRUCTURES_NEAREST_NEIGHBORS_LINEAR_

#include "ompl/datastructures/NearestNeighbors.h"

namespace ompl
{

    template<typename _T, typename _DistanceFunction>
    class NearestNeighborsLinear : public NearestNeighbors<_T, _DistanceFunction>
    {
    public:
        NearestNeighborsLinear(void) : NearestNeighbors<_T, _DistanceFunction>()
	{
	}
	
	virtual ~NearestNeighborsLinear(void)
	{
	}
	
	virtual void clear(void)
	{
	    m_data.clear();
	}

	virtual void add(_T &data)
	{
	    m_data.push_back(data);	    
	}
	
	virtual _T nearest(_T &data) const
	{
	    int pos = -1;
	    double dmin = 0.0;
	    for (unsigned int i = 0 ; i < m_data.size() ; ++i)
	    {
		double distance = _DistanceFunction()(m_data[i], data, NearestNeighbors<_T, _DistanceFunction>::m_parameter);
		if (pos < 0 || dmin > distance)
		{
		    pos = i;
		    dmin = distance;
		}		
	    }
	    return pos >= 0 ? m_data[pos] : data;
	}
	
	virtual unsigned int size(void) const
	{
	    return m_data.size();
	}
	
	virtual void list(std::vector<_T> &data) const
	{
	    data = m_data;
	}
	
    protected:
	
	std::vector<_T>  m_data;
	
    };
    
    
}

#endif
