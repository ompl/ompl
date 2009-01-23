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

/** \author Ioan Sucan, Morgan Quigley */

#ifndef OMPL_RANDOM_UTILS_
#define OMPL_RANDOM_UTILS_

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
	
	/** Initialize random number generator */
	void init(rngState *state);
	
	/** Uniform random number generator */	
	double uniform(rngState *state, double lower_bound = 0.0, double upper_bound = 1.0);
	int    uniformInt(rngState *state, int lower_bound, int upper_bound);
	bool   uniformBool(rngState *state);  
	
	/** Gaussian random number generator */	
	double gaussian(rngState *state, double mean, double stddev);
	double bounded_gaussian(rngState *state, double mean, double stddev, double max_stddev);
	
	/** Random quaternion generator. The returned value has the order (x,y,z,w) */	
	void quaternion(rngState* state, double value[4]);
	
    }
}

#endif

