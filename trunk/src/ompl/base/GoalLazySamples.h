/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2010, Rice University
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
*   * Neither the name of the Rice University nor the names of its
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

#ifndef OMPL_BASE_GOAL_LAZY_SAMPLES_
#define OMPL_BASE_GOAL_LAZY_SAMPLES_

#include "ompl/base/GoalStates.h"
#include <boost/thread/thread.hpp>
#include <boost/function.hpp>
#include <limits>

namespace ompl
{
    
    namespace base
    {
	
	/** \brief Definition of a goal region that can be sampled,
	 but the sampling process can be slow.  This class allows
	 sampling the happen in a separate thread, and the number of
	 goals may increase, as the planner is running, in a
	 thread-safe manner. */
	class GoalLazySamples : public GoalStates
	{
	public:

	    /** \brief Create a goal region that can be sampled in a
		lazy fashion. A function that produces samples from
		that region needs to be passed to this
		constructor. The sampling thread is automatically
		started if \e autoStart is true. 

                The function \e samplerFunc returns a truth value. If
		the return value is true, further calls to the
		function can be made. If the return is false, no more
		calls should be made. The function takes two
		arguments: the instance of GoalLazySamples making the
		call and the state to fill with a goal state. For
		every sampled state, addStateIfDifferent() is called. */
	    GoalLazySamples(const SpaceInformationPtr &si, const boost::function2<bool, const GoalLazySamples*, State*> &samplerFunc, bool autoStart = true);

	    virtual ~GoalLazySamples(void);
	    
	    virtual void sampleGoal(State *st) const;
	    
	    virtual double distanceGoal(const State *st) const;
	    
	    virtual void addState(const State* st);
            
            /** \brief Start the goal sampling thread */
            void startSampling(void);
            
            /** \brief Stop the goal sampling thread */
            void stopSampling(void);
            
            /** \brief Return true of maxSampleCount() > 0 or if the
                sampling thread is active, as in this case it is
                possible a sample can be produced. */
            virtual bool canSample(void) const;
            
            /** \brief Add a state \e st if it further away that \e minDistance from previously added states. */
            void addStateIfDifferent(const State* st, double minDistance = std::numeric_limits<double>::epsilon());

	    virtual void clear(void);
	    
	protected:
	    
	    /** \brief The function that samples goals by calling \e samplerFunc_ in a separate thread */
	    void goalSamplingThread(void);
	    
	    /** \brief Lock for updating the set of states */
	    mutable boost::mutex           lock_;
	    
	    /** \brief Function that produces samples */
	    boost::function2<bool, const GoalLazySamples*, State*>
                                           samplerFunc_;
	    
	    /** \brief Flag used to notify the sampling thread to terminate sampling */
	    bool                           terminateSamplingThread_;
	    
	    /** \brief Additional thread for sampling goal states */
	    boost::thread                 *samplingThread_;
	    
	};
	
    }
}

#endif
