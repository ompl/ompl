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

/** \author Ioan Sucan  */

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <ctime>
#include <climits>
#include "ompl/base/util/random_utils.h"

static std::vector<ompl::random_utils::RNG> RNGSET_STATES(1);

ompl::random_utils::RNG::RNG(void)
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
    m_state.seed = s;
    m_state.gaussian.valid = false;
}

double ompl::random_utils::RNG::uniform(double lower_bound, 
					double upper_bound)
{
    return (upper_bound - lower_bound) 
	* (double)rand_r(&m_state.seed) / ((double)(RAND_MAX) + 1.0)
	+ lower_bound;     
}

int ompl::random_utils::RNG::uniformInt(int lower_bound, int upper_bound)
{
    return (int)uniform((double)lower_bound, 
			(double)(upper_bound + 1));
}

bool ompl::random_utils::RNG::uniformBool(void)
{
    return uniform(0.0, 1.0) <= 0.5;
}

double ompl::random_utils::RNG::gaussian(double mean, double stddev)
{
    if (m_state.gaussian.valid)
    {
	double r = m_state.gaussian.last * stddev + mean;
	m_state.gaussian.valid = false;
	return r;
    }
    else
    {	
	double x1, x2, w;
	do
	{
	    x1 = uniform(-1.0, 1.0);
	    x2 = uniform(-1.0, 1.0);
	    w = x1 * x1 + x2 * x2;
	} while (w >= 1.0 || w == 0.0);
	w = sqrt(-2.0 * log(w) / w);
	m_state.gaussian.valid = true;
	m_state.gaussian.last  = x2 * w;
	return x1 * stddev * w + mean;
    }
}

double ompl::random_utils::RNG::bounded_gaussian(double mean, double stddev, double max_stddev)
{
    double sample, max_s = max_stddev * stddev;
    do
    {
	sample = gaussian(mean, stddev);
    } while (fabs(sample - mean) > max_s);
    return sample;
}

// From: "Uniform Random Rotations", Ken Shoemake, Graphics Gems III,
//       pg. 124-132
void ompl::random_utils::RNG::quaternion(double value[4])
{
    double x0 = uniform();    
    double r1 = sqrt(1.0 - x0), r2 = sqrt(x0);
    double t1 = 2.0 * M_PI * uniform(), t2 = 2.0 * M_PI * uniform();
    double c1 = cos(t1), s1 = sin(t1);
    double c2 = cos(t2), s2 = sin(t2);
    value[0] = s1 * r1;
    value[1] = c1 * r1;
    value[2] = s2 * r2;
    value[3] = c2 * r2;
}

ompl::random_utils::RNGSet::RNGSet(void)
{
    m_states = &RNGSET_STATES;
    m_threadIndex = 0;
}

unsigned int ompl::random_utils::getMaxThreads(void)
{
    return RNGSET_STATES.size();
}

void ompl::random_utils::setMaxThreads(unsigned int threads)
{
    RNGSET_STATES.resize(threads);
    // make sure we do not have the same seed for the threads 
    for (unsigned int i  = 1 ; i < RNGSET_STATES.size() ; ++i)
	RNGSET_STATES[i].m_state.seed = RNGSET_STATES[0].uniformInt(0, INT_MAX);
}
