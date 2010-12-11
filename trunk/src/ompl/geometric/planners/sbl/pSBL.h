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

/* Author: Ioan Sucan */

#ifndef OMPL_GEOMETRIC_PLANNERS_SBL_pSBL_
#define OMPL_GEOMETRIC_PLANNERS_SBL_pSBL_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/base/StateSamplerArray.h"
#include "ompl/datastructures/Grid.h"
#include <boost/thread/mutex.hpp>
#include <vector>

namespace ompl
{

    namespace geometric
    {

        /**
           @anchor gpSBL

           @par Short description

           SBL is a tree-based motion planner that attempts to grow two
           trees at once: one grows from the starting state and the other
           from the goal state. Attempts are made to connect these trees
           at every step of the expansion. If they are connected, a
           solution path is obtained. However, this solution path is not
           certain to be valid (the lazy part of the algorithm) so it is
           checked for validity. If invalid parts are found, they are
           removed from the tree and exploration of the state space
           continues until a solution is found.

           To guide the exploration, and additional grid data structure is
           maintained. Grid cells contain states that have been previously
           visited. When deciding which state to use for further
           expansion, this grid is used and least filled grid cells have
           most chances of being selected. The grid is usually imposed on
           a projection of the state space. This projection needs to be
           set before using the planner.

           @par External documentation
           G. Sánchez and J.-C. Latombe, A single-query bi-directional probabilistic roadmap planner with lazy collision checking, in <em>The Tenth International Symposium on Robotics Research</em>, pp. 403–417, 2001.
           DOI: <a href="http://dx.doi.org/10.1007/3-540-36460-9_27">10.1007/3-540-36460-9_27</a><br>
           <a href="http://www.springerlink.com/content/9843341054386hh6/fulltext.pdf">[PDF]</a>
        */


        /** \brief Parallel Single-query Bi-directional Lazy collision checking planner */
        class pSBL : public base::Planner
        {
        public:

            pSBL(const base::SpaceInformationPtr &si) : base::Planner(si, "pSBL"),
                                                        samplerArray_(si)
            {
                type_ = base::PLAN_TO_GOAL_STATE;
                maxDistance_ = 0.0;
                setThreadCount(2);
            }

            virtual ~pSBL(void)
            {
                freeMemory();
            }

            /** \brief Set the projection evaluator. This class is
                able to compute the projection of a given state. */
            void setProjectionEvaluator(const base::ProjectionEvaluatorPtr &projectionEvaluator)
            {
                projectionEvaluator_ = projectionEvaluator;
            }

            /** \brief Set the projection evaluator (select one from
                the ones registered with the state manifold). */
            void setProjectionEvaluator(const std::string &name)
            {
                projectionEvaluator_ = si_->getStateManifold()->getProjection(name);
            }

            /** \brief Get the projection evaluator. */
            const base::ProjectionEvaluatorPtr& getProjectionEvaluator(void) const
            {
                return projectionEvaluator_;
            }

            /** \brief Set the range the planner is supposed to use.

                This parameter greatly influences the runtime of the
                algorithm. It represents the maximum length of a
                motion to be added in the tree of motions. */
            void setRange(double distance)
            {
                maxDistance_ = distance;
            }

            /** \brief Get the range the planner is using */
            double getRange(void) const
            {
                return maxDistance_;
            }

            /** \brief Set the number of threads the planner should use. Default is 2. */
            void setThreadCount(unsigned int nthreads);

            /** \brief Get the thread count */
            unsigned int getThreadCount(void) const
            {
                return threadCount_;
            }

            virtual void setup(void);

            virtual bool solve(const base::PlannerTerminationCondition &ptc);

            virtual void clear(void);

            virtual void getPlannerData(base::PlannerData &data) const;

        protected:

            class Motion;
            typedef std::vector<Motion*> MotionSet;

            class Motion
            {
            public:

                Motion(void) : root(NULL), state(NULL), parent(NULL), valid(false)
                {
                }

                Motion(const base::SpaceInformationPtr &si) : root(NULL), state(si->allocState()), parent(NULL), valid(false)
                {
                }

                ~Motion(void)
                {
                }

                const base::State *root;
                base::State       *state;
                Motion            *parent;
                bool               valid;
                MotionSet          children;
                boost::mutex       lock;
            };

            struct TreeData
            {
                TreeData(void) : grid(0), size(0)
                {
                }

                Grid<MotionSet> grid;
                unsigned int    size;
                boost::mutex    lock;
            };

            struct SolutionInfo
            {
                std::vector<Motion*> solution;
                bool                 found;
                boost::mutex         lock;
            };

            struct PendingRemoveMotion
            {
                TreeData *tree;
                Motion   *motion;
            };

            struct MotionsToBeRemoved
            {
                std::vector<PendingRemoveMotion> motions;
                boost::mutex                     lock;
            };

            void threadSolve(unsigned int tid, const base::PlannerTerminationCondition &ptc, SolutionInfo *sol);

            void freeMemory(void)
            {
                freeGridMotions(tStart_.grid);
                freeGridMotions(tGoal_.grid);
            }

            void freeGridMotions(Grid<MotionSet> &grid);

            void addMotion(TreeData &tree, Motion *motion);
            Motion* selectMotion(RNG &rng, TreeData &tree);
            void removeMotion(TreeData &tree, Motion *motion, std::map<Motion*, bool> &seen);
            bool isPathValid(TreeData &tree, Motion *motion);
            bool checkSolution(RNG &rng, bool start, TreeData &tree, TreeData &otherTree, Motion *motion, std::vector<Motion*> &solution);


            base::StateSamplerArray<base::SAMPLER_VALID> samplerArray_;
            base::ProjectionEvaluatorPtr                 projectionEvaluator_;

            TreeData                                     tStart_;
            TreeData                                     tGoal_;

            MotionsToBeRemoved                           removeList_;
            boost::mutex                                 loopLock_;
            boost::mutex                                 loopLockCounter_;
            unsigned int                                 loopCounter_;

            double                                       maxDistance_;

            unsigned int                                 threadCount_;
        };

    }
}

#endif
