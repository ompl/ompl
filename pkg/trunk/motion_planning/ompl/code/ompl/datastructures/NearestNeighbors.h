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

/* \author Ioan Sucan */

#ifndef OMPL_DATASTRUCTURES_NEAREST_NEIGHBORS_
#define OMPL_DATASTRUCTURES_NEAREST_NEIGHBORS_

#include <vector>
#include <boost/bind.hpp>

namespace ompl
{
    
    template<typename _T>
    class NearestNeighbors
    {
    public:
	
	typedef	boost::function<double(const _T &a, const _T &b)> DistanceFunction;
	
	NearestNeighbors(void)
	{
	}
	
	virtual ~NearestNeighbors(void)
	{
	}

	void setDistanceFunction(const DistanceFunction &distFun)
	{
	    m_distFun = distFun;
	}

	DistanceFunction& getDistanceFunction(void) const
	{
	    return m_distFun;
	}
	
	virtual void clear(void) = 0;
	virtual void add(_T &data) = 0;
	virtual bool remove(_T &data) = 0;
	virtual _T nearest(_T &data) const = 0;
	virtual unsigned int size(void) const = 0;		
	virtual void list(std::vector<_T> &data) const = 0;
		
    protected:
	
	DistanceFunction m_distFun;
	
    };
    
    
}

#endif
