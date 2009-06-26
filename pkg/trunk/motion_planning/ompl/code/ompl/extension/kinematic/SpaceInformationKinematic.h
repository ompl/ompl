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

#ifndef OMPL_EXTENSION_KINEMATIC_SPACE_INFORMATION_KINEMATIC_
#define OMPL_EXTENSION_KINEMATIC_SPACE_INFORMATION_KINEMATIC_

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/StateDistanceEvaluator.h"
#include "ompl/extension/kinematic/PathKinematic.h"
#include "ompl/extension/kinematic/GoalKinematic.h"
#include <vector>
#include <valarray>

/** Main namespace */
namespace ompl
{

    namespace kinematic
    {
	
	/** Space information useful for kinematic planning */
	class SpaceInformationKinematic : public base::SpaceInformation
	{
	public:
	    
	    /** Constructor; setup() needs to be called as well, before use */
	    SpaceInformationKinematic(void) : base::SpaceInformation(),
					      m_defaultDistanceEvaluator(dynamic_cast<base::SpaceInformation*>(this))
	    {
		m_stateDistanceEvaluator = &m_defaultDistanceEvaluator;
	    }
	    
	    /** Destructor */
	    virtual ~SpaceInformationKinematic(void)
	    {
	    }
	    
	    /** A class that can perform sampling. Usually an instance of this class is needed
	     * for sampling states */
	    class SamplingCore
	    {	    
	    public:
		SamplingCore(const SpaceInformationKinematic *si) : m_si(si) 
		{
		}	    
		
		virtual ~SamplingCore(void)
		{
		}
		
		/** Sample a state */
		virtual void sample(base::State *state);
		
		/** Sample a state near another, within given bounds */
		virtual void sampleNear(base::State *state, const base::State *near, const double rho);
		
		/** Sample a state near another, within given bounds */
		virtual void sampleNear(base::State *state, const base::State *near, const std::vector<double> &rho);
		
	    protected:
		
		const SpaceInformationKinematic *m_si;	    
		random_utils::RNG                m_rng;
	    };
	    
	    
	    /** Find a valid state near a given one. If the given state is valid, it will be returned itself.
	     *  The two passed state pointers must point to different states. Returns true on success.  */
	    bool searchValidNearby(base::State *state, const base::State *near, const std::vector<double> &rho, unsigned int attempts) const;

	    /** Many times the start or goal state will barely touch an obstacle. In this case, we may want to automaticaly
	      * find a neaby state that is valid so motion planning can be performed. This function enables this behaviour.
	      * The allowed distance (per state component) for both start and goal states is specified. The number of attempts
	      * is also specified */
	    void fixInvalidInputStates(const std::vector<double> &rhoStart, const std::vector<double> &rhoGoal, unsigned int attempts);
	    
	    /** Check if the path between two motions is valid using subdivision */
	    bool checkMotionSubdivision(const base::State *s1, const base::State *s2) const;

	    /** Incrementally check if the path between two motions is valid */
	    bool checkMotionIncremental(const base::State *s1, const base::State *s2,
					base::State *lastValidState = NULL, double *lastValidTime = NULL) const;
	    
	    /** Get the states that make up a motion. Returns the number of states that were added */
	    unsigned int getMotionStates(const base::State *s1, const base::State *s2, std::vector<base::State*> &states, bool alloc) const;
	    
	    /** Check if the path is valid */
	    bool checkPath(const PathKinematic *path) const;
	
	    /** Insert states in a path, at the collision checking resolution */
	    void interpolatePath(PathKinematic *path, double factor = 1.0) const;
	
	    /** Perform additional tasks to finish the initialization of
		the space information */
	    virtual void setup(void);
	    
	protected:
	    
	    /** For functions that need to interpolate between two states, find the appropriate step size */
	    int findDifferenceStep(const base::State *s1, const base::State *s2, double factor,
				   std::valarray<double> &step) const;
	    
	private:
	    
	    base::L2SquareStateDistanceEvaluator m_defaultDistanceEvaluator;
	    
	};
    }
    
}

#endif
