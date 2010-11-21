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

        /** \brief Wrapper to plan in a decoupled fashion for problems with geometric constraints. 

            This class behaves exactly like a planner, but it allows
            computation of motion plans in different subspaces of the
            state space. The state manifold for this planner @b must
            be a ompl::base::CompoundStateManifold. This is called the
            \e full state manifold. Planning is done in the
            submanifolds of the full manifold (these are called \e
            partial manifolds). By default, this planner attempts to
            split the input full manifold into two partial manifolds
            of similar dimension. For example,
            ompl::base::SE3StateManifold would get split automatically
            in ompl::base::RealVectorStateManifold and
            ompl::base::SO3StateManifold. The partial manifolds must
            have no common dimensions and their Cartesian product must
            be the same as for the full manifold. 
            
            \note Not all start states may be usable. For instance, if
            four given start states can be split with respect to two
            partial manifolds as [A,1], [B,1], [C,2], [D,1], then only
            the start states [A,1], [B,1] and [D,1] would be used. The
            reasoning behind this is that planning in the first
            partial manifold assumes there are no changes in the state
            for subsequent partial manifolds. The largest subset from
            the starting states is selected such that this constraint
            holds true. The states that are not used for planning can
            be retrieved by calling  getRejectedStartStates(). */
        class DecoupledPlanner : public base::Planner
        {
        public:
            	    
	    /** \brief Constructor */
	    DecoupledPlanner(const base::SpaceInformationPtr &si);
	    
	    virtual ~DecoupledPlanner(void)
	    {
	    }

            /** \brief Since planning is performed in sub-manifolds,
                this function does not set any states or motions */
	    virtual void getPlannerData(base::PlannerData &data) const;
            
	    virtual bool solve(const base::PlannerTerminationCondition &ptc);
	    
	    virtual void clear(void);
	    
	    virtual void setup(void);
	    
            /** \brief Add a configured planner to plan for one of the
                partial state manifolds. This planner must have a
                state validation routine set. The problem definition
                should not be supplied. */
            void addComponent(const base::PlannerPtr &planner);
            
            /** \brief Designate a partial manifold to perform
                planning in. Default settings are assumed for the
                space information and a state validity checker is
                automatically defined, based on the state validity
                checker for the full manifold. */
            void addComponent(const base::StateManifoldPtr &manifold, const base::PlannerAllocator &pa = base::PlannerAllocator());
            
            /** \brief Automatically split the full manifold into a maximum number of parts (\e componnets) */
            void setComponentCount(unsigned int components);
            
            /** \brief Clear the set of partial manifolds used by the planner */
            void clearComponents(void);

            /** \brief Get the set of start states that were not considered for planning */
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
            
            /** \brief Extract valid start & goal states */
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
