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
#include "ompl/datastructures/PDF.h"
#include <thread>
#include <mutex>
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
           from the goal state. The tree expansion strategy is the same as for \ref gEST "EST".
           Attempts are made to connect these trees
           at every step of the expansion. If they are connected, a
           solution path is obtained. However, this solution path is not
           certain to be valid (the lazy part of the algorithm) so it is
           checked for validity. If invalid parts are found, they are
           removed from the tree and exploration of the state space
           continues until a solution is found.
           To guide the exploration, an additional grid data
           structure is maintained. Grid cells contain states that
           have been previously visited. When deciding which state to
           use for further expansion, this grid is used;
           least-filled grid cells have most chances of being selected. The
           grid is usually imposed on a projection of the state
           space. This projection needs to be set before using the
           planner (setProjectionEvaluator() function). Connection of states in different trees is
           attempted if they fall in the same grid cell. If no projection is
           set, the planner will attempt to use the default projection
           associated to the state space. An exception is thrown if
           no default projection is available either.
           @par External documentation
           G. Sánchez and J.-C. Latombe, A single-query bi-directional probabilistic roadmap planner with lazy collision
           checking, in <em>The Tenth International Symposium on Robotics Research</em>, pp. 403–417, 2001.
           DOI: [10.1007/3-540-36460-9_27](http://dx.doi.org/10.1007/3-540-36460-9_27)<br>
           [[PDF]](http://www.springerlink.com/content/9843341054386hh6/fulltext.pdf)</a>
        */

        /** \brief Parallel Single-query Bi-directional Lazy collision checking planner */
        class pSBL : public base::Planner
        {
        public:
            pSBL(const base::SpaceInformationPtr &si);

            ~pSBL() override;

            /** \brief Set the projection evaluator. This class is
                able to compute the projection of a given state. */
            void setProjectionEvaluator(const base::ProjectionEvaluatorPtr &projectionEvaluator)
            {
                projectionEvaluator_ = projectionEvaluator;
            }

            /** \brief Set the projection evaluator (select one from
                the ones registered with the state space). */
            void setProjectionEvaluator(const std::string &name)
            {
                projectionEvaluator_ = si_->getStateSpace()->getProjection(name);
            }

            /** \brief Get the projection evaluator. */
            const base::ProjectionEvaluatorPtr &getProjectionEvaluator() const
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
            double getRange() const
            {
                return maxDistance_;
            }

            /** \brief Set the number of threads the planner should use. Default is 2. */
            void setThreadCount(unsigned int nthreads);

            /** \brief Get the thread count */
            unsigned int getThreadCount() const
            {
                return threadCount_;
            }

            void setup() override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void clear() override;

            void getPlannerData(base::PlannerData &data) const override;

        protected:
            class Motion;
            struct MotionInfo;

            /** \brief A grid cell */
            using GridCell = Grid<MotionInfo>::Cell;

            /** \brief A PDF of grid cells */
            using CellPDF = PDF<GridCell *>;

            class Motion
            {
            public:
                Motion() = default;

                Motion(const base::SpaceInformationPtr &si)
                  : state(si->allocState())
                {
                }

                ~Motion() = default;

                const base::State *root{nullptr};
                base::State *state{nullptr};
                Motion *parent{nullptr};
                bool valid{false};
                std::vector<Motion *> children;
                std::mutex lock;
            };

            /** \brief A struct containing an array of motions and a corresponding PDF element */
            struct MotionInfo
            {
                Motion *operator[](unsigned int i)
                {
                    return motions_[i];
                }
                std::vector<Motion *>::iterator begin()
                {
                    return motions_.begin();
                }
                void erase(std::vector<Motion *>::iterator iter)
                {
                    motions_.erase(iter);
                }
                void push_back(Motion *m)
                {
                    motions_.push_back(m);
                }
                unsigned int size() const
                {
                    return motions_.size();
                }
                bool empty() const
                {
                    return motions_.empty();
                }
                std::vector<Motion *> motions_;
                CellPDF::Element *elem_;
            };

            struct TreeData
            {
                TreeData() = default;

                Grid<MotionInfo> grid{0};
                unsigned int size{0};
                CellPDF pdf;
                std::mutex lock;
            };

            struct SolutionInfo
            {
                std::vector<Motion *> solution;
                bool found;
                std::mutex lock;
            };

            struct PendingRemoveMotion
            {
                TreeData *tree;
                Motion *motion;
            };

            struct MotionsToBeRemoved
            {
                std::vector<PendingRemoveMotion> motions;
                std::mutex lock;
            };

            void threadSolve(unsigned int tid, const base::PlannerTerminationCondition &ptc, SolutionInfo *sol);

            void freeMemory()
            {
                freeGridMotions(tStart_.grid);
                freeGridMotions(tGoal_.grid);
            }

            void freeGridMotions(Grid<MotionInfo> &grid);

            void addMotion(TreeData &tree, Motion *motion);
            Motion *selectMotion(RNG &rng, TreeData &tree);
            void removeMotion(TreeData &tree, Motion *motion, std::map<Motion *, bool> &seen);
            bool isPathValid(TreeData &tree, Motion *motion);
            bool checkSolution(RNG &rng, bool start, TreeData &tree, TreeData &otherTree, Motion *motion,
                               std::vector<Motion *> &solution);

            base::StateSamplerArray<base::ValidStateSampler> samplerArray_;
            base::ProjectionEvaluatorPtr projectionEvaluator_;

            TreeData tStart_;
            TreeData tGoal_;

            MotionsToBeRemoved removeList_;
            std::mutex loopLock_;
            std::mutex loopLockCounter_;
            unsigned int loopCounter_;

            double maxDistance_{0.};

            unsigned int threadCount_;

            /** \brief The pair of states in each tree connected during planning.  Used for PlannerData computation */
            std::pair<base::State *, base::State *> connectionPoint_{nullptr, nullptr};
        };
    }
}

#endif
