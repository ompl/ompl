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

/* Author: Ioan Sucan */

#ifndef OMPL_DATASTRUCTURES_NEAREST_NEIGHBORS_LINEAR_
#define OMPL_DATASTRUCTURES_NEAREST_NEIGHBORS_LINEAR_

#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/util/Exception.h"
#include <algorithm>

namespace ompl
{

    /** \brief A nearest neighbors datastructure that uses linear
	search.
	
	\li Search for nearest neighbor is O(n). 
	\li Search for k-nearest neighbors is  O(n log(k)).
	\li Search for neighbors within a range is O(n log(n)).
	\li Adding an element to the datastructure is O(1).
	\li Removing an element from the datastructure O(n).
    */
    template<typename _T>
    class NearestNeighborsLinear : public NearestNeighbors<_T>
    {
    public:
        NearestNeighborsLinear(void) : NearestNeighbors<_T>()
	{
	}
	
	virtual ~NearestNeighborsLinear(void)
	{
	}
	
	virtual void clear(void)
	{
	    data_.clear();
	}

	virtual void add(_T &data)
	{
	    data_.push_back(data);
	}

	virtual bool remove(_T &data)
	{
	    for (int i = data_.size() - 1 ; i >= 0 ; --i)
		if (data_[i] == data)
		{
		    data_.erase(data_.begin() + i);
		    return true;
		}
	    return false;
	}
	
	virtual _T nearest(const _T &data) const
	{
	    int pos = -1;
	    double dmin = 0.0;
	    for (unsigned int i = 0 ; i < data_.size() ; ++i)
	    {
		double distance = NearestNeighbors<_T>::distFun_(data_[i], data);
		if (pos < 0 || dmin > distance)
		{
		    pos = i;
		    dmin = distance;
		}
	    }
	    if (pos >= 0) 
		return data_[pos];
	    
	    throw Exception("No elements found");
	}

	virtual void nearestK(const _T &data, unsigned int k, std::vector<_T> &nbh) const
	{
	    nbh = data_;
	    if (nbh.size() > k)
	    {
		std::partial_sort(nbh.begin(), nbh.begin() + k, nbh.end(),
				  MySort(data, NearestNeighbors<_T>::distFun_));
		nbh.resize(k);
	    }
	    else
	    {
		std::sort(nbh.begin(), nbh.end(), MySort(data, NearestNeighbors<_T>::distFun_));
	    }
	}

	virtual void nearestR(const _T &data, double radius, std::vector<_T> &nbh) const
	{
	    nbh.clear();
	    for (unsigned int i = 0 ; i < data_.size() ; ++i)
		if (NearestNeighbors<_T>::distFun_(data_[i], data) <= radius)
		    nbh.push_back(data_[i]);
	    std::sort(nbh.begin(), nbh.end(), MySort(data, NearestNeighbors<_T>::distFun_));
	}
	
	virtual std::size_t size(void) const
	{
	    return data_.size();
	}
	
	virtual void list(std::vector<_T> &data) const
	{
	    data = data_;
	}
	
    protected:

	/** \brief The data elements stored in this structure */
	std::vector<_T>   data_;
	
    private:
	
	struct MySort
	{
	    MySort(const _T &e, const typename NearestNeighbors<_T>::DistanceFunction &df) : e_(e), df_(df)
	    {
	    }
	    
	    bool operator()(const _T &a, const _T &b) const
	    {
		return df_(a, e_) < df_(b, e_);
	    }

	    const _T                                              &e_;
	    const typename NearestNeighbors<_T>::DistanceFunction &df_;
	};

    };
    
    
}

#endif
