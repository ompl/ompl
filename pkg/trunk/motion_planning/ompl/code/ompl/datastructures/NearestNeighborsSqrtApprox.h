#ifndef OMPL_DATASTRUCTURES_NEAREST_NEIGHBORS_SQRT_APPROX_
#define OMPL_DATASTRUCTURES_NEAREST_NEIGHBORS_SQRT_APPROX_

#include "ompl/datastructures/NearestNeighbors.h"
#include <cmath>

namespace ompl
{

    template<typename _T, typename _DistanceFunction>
    class NearestNeighborsSqrtApprox : public NearestNeighbors<_T, _DistanceFunction>
    {
    public:
        NearestNeighborsSqrtApprox(void) : NearestNeighbors<_T, _DistanceFunction>()
	{
	    m_checks = 0;
	}
	
	virtual ~NearestNeighborsSqrtApprox(void)
	{
	}
	
	virtual void clear(void)
	{
	    m_data.clear();
	    m_active.clear();
	}

	virtual void add(_T &data)
	{
	    m_data.push_back(data);
	    m_active.push_back(true);
	    m_checks = 1 + (int)floor(sqrt(m_data.size()));
	}

	virtual bool remove(_T &data)
	{
	    for (int i = m_data.size() - 1 ; i >= 0 ; --i)
		if (m_data[i] == data)
		{
		    m_active[i] = false;
		    return true;
		}
	    return false;
	}
	
	virtual _T nearest(_T &data) const
	{
	    int pos = -1;
	    if (m_checks > 0)
	    {
		double dmin = 0.0;
		unsigned int n = m_data.size();
		unsigned int offset = reinterpret_cast<unsigned long>(&data) % m_checks;
		for (unsigned int j = 0 ; j < m_checks ; ++j)
		{
		    unsigned int i = (j * m_checks + offset) % n;
		    if (m_active[i])
		    {
			double distance = _DistanceFunction()(m_data[i], data, NearestNeighbors<_T, _DistanceFunction>::m_parameter);
			if (pos < 0 || dmin > distance)
			{
			    pos = i;
			    dmin = distance;
			}
		    }
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
	
	std::vector<_T>   m_data;
	std::vector<bool> m_active;
	unsigned int      m_checks;
    };
    
    
}

#endif
