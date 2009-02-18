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

/** \author Ioan Sucan */

#ifndef OMPL_BASE_UTIL_RANDOM_UTILS_
#define OMPL_BASE_UTIL_RANDOM_UTILS_

#include <vector>
#include <boost/thread/mutex.hpp>

namespace ompl
{
    namespace random_utils
    {

	/** Random number generator state */
	struct rngState
	{
	    unsigned int seed;
	    struct
	    {
		double last;
		bool   valid;
	    } gaussian;
	};
	
	/** Random number generation based on a state */
	class RNG
	{
	    friend class RNGSet;
	    
	public:
	    
	    RNG(void);
	    
	    /** Uniform random number generator */	
	    double uniform(double lower_bound = 0.0, double upper_bound = 1.0);
	    int    uniformInt(int lower_bound, int upper_bound);
	    bool   uniformBool(void);  
	    
	    /** Gaussian random number generator */	
	    double gaussian(double mean, double stddev);
	    double boundedGaussian(double mean, double stddev, double max_stddev);
	    double halfNormal(double r_min, double r_max, double focus = 3.0);
	    int halfNormalInt(int r_min, int r_max, double focus = 3.0);
	    
	    /** Random quaternion generator. The returned value has the order (x,y,z,w) */	
	    void quaternion(double value[4]);
	    
	private:
	    
	    rngState m_state;
	};
	
	/** This class is thread-safe as long as the maximum number of threads is set */
	class RNGSet
	{
	public:
	    
	    RNGSet(void)
	    {
	    }
	    	    
	    /** Uniform random number generator */	
	    double uniform(double lower_bound = 0.0, double upper_bound = 1.0) const
	    {
		return nextState().uniform(lower_bound, upper_bound);
	    }
	    
	    int    uniformInt(int lower_bound, int upper_bound) const
	    {
		return nextState().uniform(lower_bound, upper_bound);
	    }
	    
	    bool   uniformBool(void) const
	    {
		return nextState().uniformBool();		
	    }
	    
	    /** Gaussian random number generator */	
	    double gaussian(double mean, double stddev) const
	    {
		return nextState().gaussian(mean, stddev);
	    }
	    
	    double boundedGaussian(double mean, double stddev, double max_stddev) const
	    {
		return nextState().boundedGaussian(mean, stddev, max_stddev);
	    }	    
	    
	    double halfNormal(double r_min, double r_max, double focus = 3.0) const
	    {		
		return nextState().halfNormal(r_min, r_max, focus);
	    }

	    int halfNormalInt(int r_min, int r_max, double focus = 3.0) const
	    {		
		return nextState().halfNormalInt(r_min, r_max, focus);
	    }
	    
	    /** Random quaternion generator. The returned value has the order (x,y,z,w) */	
	    void quaternion(double value[4]) const
	    {
		return nextState().quaternion(value);
	    }
	    
	    /** Get the maximum number of threads for which the RNGSet is thread-safe */
	    static unsigned int getMaxThreads(void);
	    
	    /** Set the maximum number of threads for which the RNGSet
		should be thread-safe.  */
	    static void setMaxThreads(unsigned int threads);
	    
	private:
	    
	    RNG& nextState(void) const
	    {
		LOCK.lock();
		unsigned int index = THREADINDEX;
		THREADINDEX = (THREADINDEX + 1) % STATES.size();
		LOCK.unlock();
		return STATES.at(index);
	    }
	    
	    static std::vector<RNG> STATES;
	    static boost::mutex     LOCK;
	    static unsigned int     THREADINDEX;
	};
    }
}

#endif
