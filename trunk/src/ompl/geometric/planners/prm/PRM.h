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

#ifndef OMPL_GEOMETRIC_PLANNERS_PRM_PRM_
#define OMPL_GEOMETRIC_PLANNERS_PRM_PRM_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include <utility>
#include <vector>
#include <map>

namespace ompl
{

    namespace geometric
    {

        /**
           @anchor gPRM

           @par Short description 
           PRM is a planner that constructs a roadmap of milestones
           that approximate the connectivity of the state space. The
           milestones are valid states in the state space. Near-by
           milestones are connected by valid motions. Finding a motion
           plan that connects two given states is reduced to a
           discrete search (this implementation uses Dijskstra) in the
           roadmap.

           @par External documentation
           L.E. Kavraki, P.Švestka, J.-C. Latombe, and M.H. Overmars,
           Probabilistic roadmaps for path planning in high-dimensional configuration spaces,
           <em>IEEE Trans. on Robotics and Automation</em>, vol. 12, pp. 566–580, Aug. 1996.
           DOI: <a href="http://dx.doi.org/10.1109/70.508439">10.1109/70.508439</a><br>
           <a href="http://ieeexplore.ieee.org/ielx4/70/11078/00508439.pdf?tp=&arnumber=508439&isnumber=11078">[PDF]</a>
           <a href="http://www.kavrakilab.org/robotics/prm.html">[more]</a>

        */

        /** \brief Probabilistic RoadMap planner */
        class PRM : public base::Planner
        {
        public:

            PRM(const base::SpaceInformationPtr &si) : base::Planner(si, "PRM")
            {
                type_ = base::PLAN_TO_GOAL_SAMPLEABLE_REGION;

                maxNearestNeighbors_ = 10;
                componentCount_ = 0;
                lastStart_ = NULL;
                lastGoal_ = NULL;
            }

            virtual ~PRM(void)
            {
                freeMemory();
            }

            /** \brief Set the maximum number of neighbors for which a
                connection to will be attempted when a new milestone
                is added */
            void setMaxNearestNeighbors(unsigned int maxNearestNeighbors)
            {
                maxNearestNeighbors_ = maxNearestNeighbors;
            }

            /** \brief Get the maximum number of neighbors for which a
                connection will be attempted when a new milestone is
                added */
            unsigned int getMaxNearestNeighbors(void) const
            {
                return maxNearestNeighbors_;
            }

            virtual void getPlannerData(base::PlannerData &data) const;

            /** \brief If the user desires, the roadmap can be
                improved for a specified amount of time. The solve()
                method will also improve the roadmap, as needed.*/
            virtual void growRoadmap(double growTime);

            virtual bool solve(const base::PlannerTerminationCondition &ptc);

            /** \brief If the user desires to recompute the previously
                obtained solution, this function allows this
                functionality */
            virtual void reconstructLastSolution(void);

            virtual void clear(void);

            /** \brief Set a different nearest neighbors datastructure */
            template<template<typename T> class NN>
            void setNearestNeighbors(void)
            {
                nn_.reset(new NN<Milestone*>());
            }

            virtual void setup(void);

        protected:


            /** \brief Representation of a milestone */
            class Milestone
            {
            public:

                Milestone(void) : state(NULL), index(0)
                {
                }

                Milestone(const base::SpaceInformationPtr &si) : state(si->allocState()), index(0)
                {
                }

                ~Milestone(void)
                {
                }

                base::State            *state;
                unsigned int            index;
                unsigned long           component;
                std::vector<Milestone*> adjacent;
                std::vector<double>     costs;
            };

            void freeMemory(void);
            virtual void nearestNeighbors(Milestone *milestone, std::vector<Milestone*> &nbh);
            Milestone* addMilestone(base::State *state);
            void uniteComponents(Milestone *m1, Milestone *m2);
            void growRoadmap(const std::vector<Milestone*> &start, const std::vector<Milestone*> &goal, const base::PlannerTerminationCondition &ptc, base::State *workState);
            bool haveSolution(const std::vector<Milestone*> &start, const std::vector<Milestone*> &goal, std::pair<Milestone*, Milestone*> *endpoints = NULL);
            void constructSolution(const Milestone* start, const Milestone* goal);

            double distanceFunction(const Milestone* a, const Milestone* b) const
            {
                return si_->distance(a->state, b->state);
            }

            base::ValidStateSamplerPtr                        sampler_;
            boost::shared_ptr< NearestNeighbors<Milestone*> > nn_;
            std::vector<Milestone*>                           milestones_;
            const Milestone                                  *lastStart_;
            const Milestone                                  *lastGoal_;
            unsigned int                                      maxNearestNeighbors_;
            std::map<unsigned long, unsigned long>            componentSizes_;
            unsigned long                                     componentCount_;
            RNG                                               rng_;
        };

    }
}

#endif
