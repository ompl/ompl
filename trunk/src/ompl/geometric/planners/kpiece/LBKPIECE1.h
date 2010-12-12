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

#ifndef OMPL_GEOMETRIC_PLANNERS_KPIECE_LBKPIECE1_
#define OMPL_GEOMETRIC_PLANNERS_KPIECE_LBKPIECE1_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/datastructures/GridB.h"
#include <vector>

namespace ompl
{

    namespace geometric
    {

        /**
           @anchor gLBKPIECE1

           @par Short description
           KPIECE is a tree-based planner that uses a discretization
           (multiple levels, in general) to guide the exploration of
           the continous space. This implementation is a simplified
           one, using a single level of discretization: one grid. The
           grid is imposed on a projection of the state space. When
           exploring the space, preference is given to the boundary of
           this grid. The boundary is computed to be the set of grid
           cells that have less than 2n non-diagonal neighbors in an
           n-dimensional projection space.
           It is important to set the projection the algorithm uses
           (setProjectionEvaluator() function). If no projection is
           set, the planner will attempt to use the default projection
           associated to the state manifold. An exception is thrown if
           no default projection is available either.
           This variant of the implementation use two trees of
           exploration with lazy collision checking, hence the LB
           prefix.

           @par External documentation
           - I.A. Şucan and L.E. Kavraki, Kinodynamic motion planning by interior-exterior cell exploration,
           in <em>Workshop on the Algorithmic Foundations of Robotics</em>, Dec. 2008.<br>
           <a href="http://ioan.sucan.ro/files/pubs/wafr2008.pdf">[PDF]</a>
           - R. Bohlin and L.E. Kavraki, Path planning using lazy PRM, in <em>Proc. 2000 IEEE Intl. Conf. on Robotics and Automation</em>, pp. 521–528, 2000. DOI: <a href="http://dx.doi.org/10.1109/ROBOT.2000.844107">10.1109/ROBOT.2000.844107</a><br>
           <a href="http://ieeexplore.ieee.org/ielx5/6794/18235/00844107.pdf?tp=&arnumber=844107&isnumber=18235">[PDF]

        */

        /** \brief Lazy Bi-directional KPIECE with one level of discretization */
        class LBKPIECE1 : public base::Planner
        {
        public:

            /** \brief Constructor */
            LBKPIECE1(const base::SpaceInformationPtr &si) : base::Planner(si, "LBKPIECE1")
            {
                type_ = base::PLAN_TO_GOAL_SAMPLEABLE_REGION;

                minValidPathFraction_ = 0.5;
                selectBorderFraction_ = 0.9;
                maxDistance_ = 0.0;

                tStart_.grid.onCellUpdate(computeImportance, NULL);
                tGoal_.grid.onCellUpdate(computeImportance, NULL);
            }

            virtual ~LBKPIECE1(void)
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

            /** \brief Set the fraction of time for focusing on the
                border (between 0 and 1). This is the minimum fraction
                used to select cells that are exterior (minimum
                because if 95% of cells are on the border, they will
                be selected with 95% chance, even if this fraction is
                set to 90%)*/
            void setBorderFraction(double bp)
            {
                selectBorderFraction_ = bp;
            }

            /** \brief Set the fraction of time for focusing on the
                border (between 0 and 1). */
            double getBorderFraction(void) const
            {
                return selectBorderFraction_;
            }

            /** \brief When extending a motion, the planner can decide
                to keep the first valid part of it, even if invalid
                states are found, as long as the valid part represents
                a sufficiently large fraction from the original
                motion. This function sets the minimum acceptable
                fraction. */
            void setMinValidPathFraction(double fraction)
            {
                minValidPathFraction_ = fraction;
            }

            /** \brief Get the value of the fraction set by setMinValidPathFraction() */
            double getMinValidPathFraction(void) const
            {
                return minValidPathFraction_;
            }

            virtual void setup(void);

            virtual bool solve(const base::PlannerTerminationCondition &ptc);
            virtual void clear(void);

            virtual void getPlannerData(base::PlannerData &data) const;

        protected:

            /** \brief Representation of a motion for this algorithm */
            class Motion
            {
            public:

                Motion(void) : root(NULL), state(NULL), parent(NULL), valid(false)
                {
                }

                /** \brief Constructor that allocates memory for the state */
                Motion(const base::SpaceInformationPtr &si) : root(NULL), state(si->allocState()), parent(NULL), valid(false)
                {
                }

                ~Motion(void)
                {
                }

                /** \brief The root state (start state) that leads to this motion */
                const base::State   *root;

                /** \brief The state contained by this motion */
                base::State         *state;

                /** \brief The parent motion in the exploration tree */
                Motion              *parent;

                /** \brief Flag indicating whether this motion has been checked for validity. */
                bool                 valid;

                /** \brief The set of motions descending from the current motion */
                std::vector<Motion*> children;
            };

            /** \brief The data held by a cell in the grid of motions */
            struct CellData
            {
                CellData(void) : coverage(0.0), selections(1), score(1.0), iteration(0), importance(0.0)
                {
                }

                ~CellData(void)
                {
                }

                /** \brief The set of motions contained in this grid cell */
                std::vector<Motion*> motions;

                /** \brief A measure of coverage for this cell. For
                    this implementation, this is the sum of motion
                    lengths */
                double               coverage;

                /** \brief The number of times this cell has been
                    selected for expansion */
                unsigned int         selections;

                /** \brief A heuristic score computed based on
                    distance to goal (if available), successes and
                    failures at expanding from this cell. */
                double               score;

                /** \brief The iteration at which this cell was created */
                unsigned int         iteration;

                /** \brief The computed importance (based on other class members) */
                double               importance;
            };

            /** \brief Definintion of an operator passed to the Grid
                structure, to order cells by importance */
            struct OrderCellsByImportance
            {
                /** \brief Order function */
                bool operator()(const CellData * const a, const CellData * const b) const
                {
                    return a->importance > b->importance;
                }
            };

            /** \brief The datatype for the maintained grid datastructure */
            typedef GridB<CellData*, OrderCellsByImportance> Grid;

            /** \brief The data defining a tree of motions for this algorithm */
            struct TreeData
            {
                TreeData(void) : grid(0), size(0), iteration(1)
                {
                }

                /** \brief A grid containing motions, imposed on a
                    projection of the state space */
                Grid         grid;

                /** \brief The total number of motions (there can be
                    multiple per cell) in the grid */
                unsigned int size;

                /** \brief The number of iterations performed on this tree */
                unsigned int iteration;
            };

            /** \brief This function is provided as a calback to the
                grid datastructure to update the importance of a
                cell */
            static void computeImportance(Grid::Cell *cell, void*)
            {
                CellData &cd = *(cell->data);
                cd.importance =  cd.score / ((cell->neighbors + 1) * cd.coverage * cd.selections);
            }

            /** \brief Free all the memory allocated by this planner */
            void freeMemory(void);

            /** \brief Free the memory for the motions contained in a grid */
            void freeGridMotions(Grid &grid);

            /** \brief Free the memory for the data contained in a grid cell */
            void freeCellData(CellData *cdata);

            /** \brief Free the memory for a motion */
            void freeMotion(Motion *motion);

            /** \brief Add a motion to the grid containing motions. As
                a hint, \e dist specifies the distance to the goal
                from the state of the motion being added. The function
                Returns the number of cells created to accommodate the
                new motion (0 or 1). */
            void addMotion(TreeData &tree, Motion* motion);

            /** \brief Select a motion and the cell it is part of from
                the grid of motions. This is where preference is given
                to cells on the boundary of the grid.*/
            Motion* selectMotion(TreeData &tree);

            /** \brief Remove a motion from a tree of motions */
            void removeMotion(TreeData &tree, Motion* motion);

            /** \brief Since solutions are computed in a lazy fashion,
                once trees are connected, the solution found needs to
                be checked for validity. This function checks whether
                the reverse path from a given motion to a root is
                valid. If this is not the case, invalid motions are removed  */
            bool isPathValid(TreeData &tree, Motion* motion, base::State *temp);

            /** \brief Check if a solution can be obtained by connecting two trees using a specified motion */
            bool checkSolution(bool start, TreeData &tree, TreeData &otherTree, Motion* motion, std::vector<Motion*> &solution, base::State *temp);

            /** \brief The employed state sampler */
            base::ManifoldStateSamplerPtr              sampler_;

            /** \brief The employed projection evaluator */
            base::ProjectionEvaluatorPtr               projectionEvaluator_;

            /** \brief The start tree */
            TreeData                                   tStart_;

            /** \brief The goal tree */
            TreeData                                   tGoal_;

            /** \brief When extending a motion, the planner can decide
                to keep the first valid part of it, even if invalid
                states are found, as long as the valid part represents
                a sufficiently large fraction from the original
                motion */
            double                                     minValidPathFraction_;

            /** \brief The fraction of time to focus exploration on
                the border of the grid. */
            double                                     selectBorderFraction_;

            /** \brief The maximum length of a motion to be added to a tree */
            double                                     maxDistance_;

            /** \brief The random number generator */
            RNG                                        rng_;
        };

    }
}


#endif
