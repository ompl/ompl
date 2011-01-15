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

#ifndef OMPL_DATASTRUCTURES_NEAREST_NEIGHBORS_SQRT_APPROX_
#define OMPL_DATASTRUCTURES_NEAREST_NEIGHBORS_SQRT_APPROX_

#include "ompl/datastructures/NearestNeighborsLinear.h"
#include <algorithm>
#include <cmath>

namespace ompl
{
    /** \brief A nearest neighbors datastructure that uses linear
	search. The linear search is done over sqrt(n) elements
	only. (Every sqrt(n) elements are skipped).
	
	\li Search for nearest neighbor is O(sqrt(n)). 
	\li Search for k-nearest neighbors is  O(n log(k)).
	\li Search for neighbors within a range is O(n log(n)).
	\li Adding an element to the datastructure is O(1).
	\li Removing an element from the datastructure O(n).
    */
    template<typename _T>
    class NearestNeighborsSqrtApprox : public NearestNeighborsLinear<_T>
    {
    public:
        NearestNeighborsSqrtApprox(void) : NearestNeighborsLinear<_T>(), checks_(0)
	{
	}
	
	virtual ~NearestNeighborsSqrtApprox(void)
	{
	}
	
	virtual void clear(void)
	{
	    NearestNeighborsLinear<_T>::clear();
	    checks_ = 0;
	}

	virtual void add(_T &data)
	{
	    NearestNeighborsLinear<_T>::add(data);
	    checks_ = 1 + (int)floor(sqrt((double)NearestNeighborsLinear<_T>::data_.size()));
	}
	
	virtual _T nearest(const _T &data) const
	{
	    int pos = -1;
	    if (checks_ > 0)
	    {
		double dmin = 0.0;
		unsigned int n = NearestNeighborsLinear<_T>::data_.size();
		unsigned int offset = reinterpret_cast<unsigned long>(&data) % checks_;
		for (unsigned int j = 0 ; j < checks_ ; ++j)
		{
		    unsigned int i = (j * checks_ + offset) % n;
		    
		    double distance = NearestNeighbors<_T>::distFun_(NearestNeighborsLinear<_T>::data_[i], data);
		    if (pos < 0 || dmin > distance)
		    {
			pos = i;
			dmin = distance;
		    }
		}
	    }
	    if (pos >= 0) 
		return NearestNeighborsLinear<_T>::data_[pos];
	    
	    throw Exception("No elements found");
	}
	
    protected:
	
	/** \brief The number of checks to be performed when looking for a nearest neighbor */
	unsigned int      checks_;

    };
        
}

#endif
