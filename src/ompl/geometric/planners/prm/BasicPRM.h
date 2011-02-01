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

#ifndef OMPL_GEOMETRIC_PLANNERS_PRM_BASIC_PRM_
#define OMPL_GEOMETRIC_PLANNERS_PRM_BASIC_PRM_

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
           @anchor gBasicPRM

           @par Short description
           PRM is a planner that constructs a roadmap of milestones
           that approximate the connectivity of the state space. The
           milestones are valid states in the state space. Near-by
           milestones are connected by valid motions. Finding a motion
           plan that connects two given states is reduced to a
           discrete search (this implementation uses Dijskstra) in the
           roadmap.
           The construction process for the roadmap includes an
           expansion strategy. This expansion strategy is not included
           in this implementation, hence the name BasicPRM.

           @par External documentation
           L.E. Kavraki, P.Švestka, J.-C. Latombe, and M.H. Overmars,
           Probabilistic roadmaps for path planning in high-dimensional configuration spaces,
           <em>IEEE Trans. on Robotics and Automation</em>, vol. 12, pp. 566–580, Aug. 1996.
           DOI: <a href="http://dx.doi.org/10.1109/70.508439">10.1109/70.508439</a><br>
           <a href="http://ieeexplore.ieee.org/ielx4/70/11078/00508439.pdf?tp=&arnumber=508439&isnumber=11078">[PDF]</a>
           <a href="http://www.kavrakilab.org/robotics/prm.html">[more]</a>

        */

        /** \brief Probabilistic RoadMap planner */
        class BasicPRM : public base::Planner
        {
        public:

            /** \brief Constructor */
            BasicPRM(const base::SpaceInformationPtr &si) : base::Planner(si, "BasicPRM")
            {
                type_ = base::PLAN_TO_GOAL_SAMPLEABLE_REGION;

                maxNearestNeighbors_ = 10;
                componentCount_ = 0;
                lastStart_ = NULL;
                lastGoal_ = NULL;
            }

            virtual ~BasicPRM(void)
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

                /** \brief Automatically allocate memory for a milestone's state */
                Milestone(const base::SpaceInformationPtr &si) : state(si->allocState()), index(0)
                {
                }

                ~Milestone(void)
                {
                }

                /** \brief The state corresponding to the milestone */
                base::State            *state;

                /** \brief The index of this milestone in the array of milestones (BasicPRM::milestones_) */
                unsigned int            index;

                /** \brief The id of the connected component this milestone is part of */
                unsigned long           component;

                /** \brief The array of milestones that can be connected to with valid paths */
                std::vector<Milestone*> adjacent;

                /** \brief The cost of the edges indicated by \e adjacent */
                std::vector<double>     costs;
            };

            /** \brief Free all the memory allocated by the planner */
            void freeMemory(void);

            /** \brief Get the list of nearest neighbors (\e nbh) for a given milestone (\e milestone) */
            virtual void nearestNeighbors(Milestone *milestone, std::vector<Milestone*> &nbh);

            /** \brief Construct a milestone for a given state (\e state) and store it in the nearest neighbors data structure */
            Milestone* addMilestone(base::State *state);

            /** \brief Make two milestones (\e m1 and \e m2) be part of the same connected component. The component with fewer elements will get the id of the component with more elements. */
            void uniteComponents(Milestone *m1, Milestone *m2);

            /** \brief Randomly sample the state space, add and connect milestones in the roadmap. Stop this process when the termination condition \e ptc returns true or when any of the \e start milestones are in the same connected component as any of the \e goal milestones. Use \e workState as temporary memory. */
            void growRoadmap(const std::vector<Milestone*> &start, const std::vector<Milestone*> &goal, const base::PlannerTerminationCondition &ptc, base::State *workState);

            /** \brief Check if there exists a solution, i.e., there exists a pair of milestones such that the first is in \e start and the second is in \e goal, and the two milestones are in the same connected component. If \e endpoints is not null, that pair is recorded. */
            bool haveSolution(const std::vector<Milestone*> &start, const std::vector<Milestone*> &goal, std::pair<Milestone*, Milestone*> *endpoints = NULL);

            /** \brief Given two milestones from the same connected component, construct a path connecting them and set it as the solution */
            void constructSolution(const Milestone* start, const Milestone* goal);

            /** \brief Compute distance between two milestones (this is simply distance between the states of the milestones) */
            double distanceFunction(const Milestone* a, const Milestone* b) const
            {
                return si_->distance(a->state, b->state);
            }

            /** \brief Sampler user for generating valid samples in the state space */
            base::ValidStateSamplerPtr                        sampler_;

            /** \brief Nearest neighbors data structure */
            boost::shared_ptr< NearestNeighbors<Milestone*> > nn_;

             /** \brief Array of available milestones */
            std::vector<Milestone*>                           milestones_;

            /** \brief Array of start milestones */
            std::vector<Milestone*>                           startM_;

            /** \brief Array of goal milestones */
            std::vector<Milestone*>                           goalM_;

            /** \brief constructSolution() will set this variable to be the milestone used as the start. This is useful if multiple solution paths are to be generated. */
            const Milestone                                  *lastStart_;

            /** \brief constructSolution() will set this variable to be the milestone used as the goal. This is useful if multiple solution paths are to be generated. */
            const Milestone                                  *lastGoal_;

            /** \brief Maximum number of nearest neighbors to attempt to connect new milestones to */
            unsigned int                                      maxNearestNeighbors_;

            /** \brief Number of elements in each component */
            std::map<unsigned long, unsigned long>            componentSizes_;

            /** \brief The number of components that have been used at a point in time. There is no component with id larger than this value, but it is not necessary for all components with smaller id to exist (they could have been merged) */
            unsigned long                                     componentCount_;

            /** \brief Random number generator */
            RNG                                               rng_;
        };

    }
}

#endif
