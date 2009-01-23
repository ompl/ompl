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

/** \author Ioan Sucan, Morgan Quigley  */

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <ctime>
#include <climits>
#include "ompl/base/util/random_utils.h"

void ompl::random_utils::init(rngState *state)
{
    FILE        *fp = fopen("/dev/urandom", "r");    
    unsigned int s;
    
    if(fp != NULL)
    {
	fread(&s, sizeof(unsigned int), 1, fp);
	fclose(fp);
    }
    else
	s = (unsigned int) time(NULL);
    state->seed = s;
    state->gaussian.valid = false;
}

double ompl::random_utils::uniform(rngState *state, double lower_bound, 
				   double upper_bound)
{
    return (upper_bound - lower_bound) 
	* (double)rand_r(&state->seed) / ((double)(RAND_MAX) + 1.0)
	+ lower_bound;     
}

int ompl::random_utils::uniformInt(rngState *state, int lower_bound, int upper_bound)
{
    return (int)random_utils::uniform(state, (double)lower_bound, 
				      (double)(upper_bound + 1));
}

bool ompl::random_utils::uniformBool(rngState *state)
{
    return uniform(state, 0.0, 1.0) <= 0.5;
}

double ompl::random_utils::gaussian(rngState *state, double mean, double stddev)
{
    if (state->gaussian.valid)
    {
	double r = state->gaussian.last * stddev + mean;
	state->gaussian.valid = false;
	return r;
    }
    else
    {	
	double x1, x2, w;
	do
	{
	    x1 = uniform(state, -1.0, 1.0);
	    x2 = uniform(state, -1.0, 1.0);
	    w = x1 * x1 + x2 * x2;
	} while (w >= 1.0 || w == 0.0);
	w = sqrt(-2.0 * log(w) / w);
	state->gaussian.valid = true;
	state->gaussian.last  = x2 * w;
	return x1 * stddev * w + mean;
    }
}

double ompl::random_utils::bounded_gaussian(rngState *state, double mean, 
					    double stddev, double max_stddev)
{
    double sample, max_s = max_stddev * stddev;
    do
    {
	sample = gaussian(state, mean, stddev);
    } while (fabs(sample - mean) > max_s);
    return sample;
}

// From: "Uniform Random Rotations", Ken Shoemake, Graphics Gems III,
//       pg. 124-132
void ompl::random_utils::quaternion(rngState* state, double value[4])
{
    double x0 = uniform(state);    
    double r1 = sqrt(1.0 - x0), r2 = sqrt(x0);
    double t1 = 2.0 * M_PI * uniform(state), t2 = 2.0 * M_PI * uniform(state);
    double c1 = cos(t1), s1 = sin(t1);
    double c2 = cos(t2), s2 = sin(t2);
    value[0] = s1 * r1;
    value[1] = c1 * r1;
    value[2] = s2 * r2;
    value[3] = c2 * r2;
}
