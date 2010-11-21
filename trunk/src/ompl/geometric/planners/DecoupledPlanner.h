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

#ifndef OMPL_GEOMETRIC_DECOUPLED_PLANNER_
#define OMPL_GEOMETRIC_DECOUPLED_PLANNER_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/base/ScopedState.h"

namespace ompl
{
    namespace geometric
    {

        class DecoupledPlanner : public base::Planner
        {
        public:
            	    
	    /** \brief Constructor */
	    DecoupledPlanner(const base::SpaceInformationPtr &si);
	    
	    virtual ~DecoupledPlanner(void)
	    {
	    }

	    virtual void getPlannerData(base::PlannerData &data) const;

	    virtual bool solve(const base::PlannerTerminationCondition &ptc);
	    
	    virtual void clear(void);
	    
	    virtual void setup(void);
	    
            void addComponent(const base::PlannerPtr &planner);
            
            void addComponent(const base::StateManifoldPtr &manifold, const base::PlannerAllocator &pa = base::PlannerAllocator());

            void setComponentCount(unsigned int components);
            
            void clearComponents(void);

            const std::vector< base::ScopedState<> > & getRejectedStartStates(void) const
            {
                return rejectedStartStates_;
            }
            
        protected:
            
            struct Component
            {
                base::PlannerPtr           planner;
                base::ProblemDefinitionPtr pdef;
            };
            
            bool processInputStates(const base::PlannerTerminationCondition &ptc);
            
            void addComponent(const base::PlannerPtr &planner, const base::ProblemDefinitionPtr &pdef);
            
            std::vector<Component>                   components_;
            base::ScopedState<>                      currentStartState_;
            unsigned int                             startStateVersion_;
            
            std::vector< base::ScopedState<> >       startStates_;
            std::vector< base::ScopedState<> >       rejectedStartStates_;
            std::vector< base::ScopedState<> >       goalStates_;
            
        };
    }
}

#endif
