/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Rice University
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

#ifndef OMPL_GEOMETRIC_PLANNERS_PRM_LAZY_PRM_
#define OMPL_GEOMETRIC_PLANNERS_PRM_LAZY_PRM_

#include "ompl/geometric/planners/prm/PRM.h"

namespace ompl
{

    namespace geometric
    {

        /**
           @anchor gLazyPRM
           @par Short description
           LazyPRM is a planner that constructs a roadmap of milestones
           that approximate the connectivity of the state space, just like PRM does.
           The difference is that the planner uses lazy collision checking.
           @par External documentation
           R. Bohlin and L.E. Kavraki
           Path Planning Using Lazy PRM
           <em>IEEE International Conference on Robotics and Automation</em>, San Francisco, pp. 521–528, 2000.
           DOI: <a href="http://dx.doi.org/10.1109/ROBOT.2000.844107">10.1109/ROBOT.2000.844107</a><br>
           <a href="http://www.kavrakilab.org/robotics/lazyprm.html">[more]</a>
        */

        /** \brief Lazy Probabilistic RoadMap planner */
        class LazyPRM : public PRM
        {
        public:

            /** \brief Constructor */
            LazyPRM(const base::SpaceInformationPtr &si, bool starStrategy = false);

            virtual ~LazyPRM(void);

            /** \brief For LazyPRM, this simply calls growRoadmap() */
            virtual void constructRoadmap(const base::PlannerTerminationCondition &ptc);

            using PRM::growRoadmap;

        protected:

            /** \brief Flag indicating validity of an edge of a vertex */
            static const unsigned int VALIDITY_UNKNOWN = 0;

            /** \brief Flag indicating validity of an edge of a vertex */
            static const unsigned int VALIDITY_TRUE    = 1;

            virtual Vertex addMilestone(base::State *state);

            virtual void growRoadmap(const base::PlannerTerminationCondition &ptc, base::State *workState);

            virtual base::PathPtr constructGeometricPath(const boost::vector_property_map<Vertex> &prev, const Vertex &start, const Vertex &goal);

            /** \brief Access the validity state of a vertex */
            boost::property_map<Graph, vertex_flags_t>::type vertexValidityProperty_;
            /** \brief Access the validity state of an edge */
            boost::property_map<Graph, edge_flags_t>::type   edgeValidityProperty_;

        };

    }
}

#endif
