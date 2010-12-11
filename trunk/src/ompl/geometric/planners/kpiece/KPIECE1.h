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

#ifndef OMPL_GEOMETRIC_PLANNERS_KPIECE_KPIECE1_
#define OMPL_GEOMETRIC_PLANNERS_KPIECE_KPIECE1_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/datastructures/GridB.h"
#include "ompl/geometric/ik/HCIK.h"
#include <vector>

namespace ompl
{

    namespace geometric
    {


        /**
           @anchor gKPIECE1

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

           @par External documentation
           I.A. Şucan and L.E. Kavraki, Kinodynamic motion planning by interior-exterior cell exploration,
           in <em>Workshop on the Algorithmic Foundations of Robotics</em>, Dec. 2008.<br>
           <a href="http://ioan.sucan.ro/files/pubs/wafr2008.pdf">[PDF]</a>
        */

        /** \brief Kinematic Planning by Interior-Exterior Cell Exploration */
        class KPIECE1 : public base::Planner
        {
        public:

            /** \brief Constructor */
            KPIECE1(const base::SpaceInformationPtr &si) : base::Planner(si, "KPIECE1"),
                                                           hcik_(si)
            {
                type_ = base::PLAN_TO_GOAL_ANY;

                goalBias_ = 0.05;
                selectBorderFraction_ = 0.9;
                badScoreFactor_ = 0.5;
                goodScoreFactor_ = 0.9;
                minValidPathFraction_ = 0.2;
                maxDistance_ = 0.0;
                tree_.grid.onCellUpdate(computeImportance, NULL);
                hcik_.setMaxImproveSteps(50);
            }

            virtual ~KPIECE1(void)
            {
                freeMemory();
            }

            virtual bool solve(const base::PlannerTerminationCondition &ptc);

            virtual void clear(void);

            /** \brief Set the goal bias.

                In the process of randomly selecting states in the state
                space to attempt to go towards, the algorithm may in fact
                choose the actual goal state, if it knows it, with some
                probability. This probability is a real number between 0.0
                and 1.0; its value should usually be around 0.05 and
                should not be too large. It is probably a good idea to use
                the default value. */
            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            /** \brief Get the goal bias the planner is using */
            double getGoalBias(void) const
            {
                return goalBias_;
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

            /** \brief Get the fraction of time to focus exploration
                on boundary */
            double getBorderFraction(void) const
            {
                return selectBorderFraction_;
            }

            /** \brief When extending a motion, the planner can decide
                to keep the first valid part of it, even if invalid
                states are found, as long as the valid part represents
                a sufficiently large fraction from the original
                motion. This function sets the minimum acceptable
                fraction (between 0 and 1). */
            void setMinValidPathFraction(double fraction)
            {
                minValidPathFraction_ = fraction;
            }

            /** \brief Get the value of the fraction set by setMinValidPathFraction() */
            double getMinValidPathFraction(void) const
            {
                return minValidPathFraction_;
            }

            /** \brief When extending a motion from a cell, the
                extension can be successful or it can fail.  If the
                extension is successful, the score of the cell is
                multiplied by \e good. If the extension fails, the
                score of the cell is multiplied by \e bad. These
                numbers should be in the range (0, 1]. */
            void setCellScoreFactor(double good, double bad)
            {
                goodScoreFactor_ = good;
                badScoreFactor_ = bad;
            }

            /** \brief Get the factor that is multiplied to a cell's
                score if extending a motion from that cell succeeded. */
            double getGoodCellScoreFactor(void) const
            {
                return goodScoreFactor_;
            }

            /** \brief Get the factor that is multiplied to a cell's
                score if extending a motion from that cell failed. */
            double getBadCellScoreFactor(void) const
            {
                return badScoreFactor_;
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

            /** \brief Get the projection evaluator */
            const base::ProjectionEvaluatorPtr& getProjectionEvaluator(void) const
            {
                return projectionEvaluator_;
            }

            virtual void setup(void);

            virtual void getPlannerData(base::PlannerData &data) const;

        protected:

            /** \brief Representation of a motion for this algorithm */
            class Motion
            {
            public:

                Motion(void) : state(NULL), parent(NULL)
                {
                }

                /** \brief Constructor that allocates memory for the state */
                Motion(const base::SpaceInformationPtr &si) : state(si->allocState()), parent(NULL)
                {
                }

                ~Motion(void)
                {
                }

                /** \brief The state contained by this motion */
                base::State       *state;

                /** \brief The parent motion in the exploration tree */
                Motion            *parent;

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
            unsigned int addMotion(Motion* motion, double dist);

            /** \brief Select a motion and the cell it is part of from
                the grid of motions. This is where preference is given
                to cells on the boundary of the grid.*/
            bool selectMotion(Motion* &smotion, Grid::Cell* &scell);

            /** \brief A manifold sampler */
            base::ManifoldStateSamplerPtr              sampler_;

            /** \brief A hill climbing algorithm used to find states
                closer to the goal */
            HCIK                                       hcik_;

            /** \brief The tree datastructure */
            TreeData                                   tree_;


            /** \brief This algorithm uses a discretization (a grid)
                to guide the exploration. The exploration is imposed
                on a projection of the state space. */
            base::ProjectionEvaluatorPtr               projectionEvaluator_;

            /** \brief When extending a motion, the planner can decide
                to keep the first valid part of it, even if invalid
                states are found, as long as the valid part represents
                a sufficiently large fraction from the original
                motion */
            double                                     minValidPathFraction_;

            /** \brief When extending a motion from a cell, the
                extension can be successful. If it is, the score of the
                cell is multiplied by this factor. */
            double                                     goodScoreFactor_;

            /** \brief When extending a motion from a cell, the
                extension can fail. If it is, the score of the cell is
                multiplied by this factor. */
            double                                     badScoreFactor_;

            /** \brief The fraction of time to focus exploration on
                the border of the grid. */
            double                                     selectBorderFraction_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is available) */
            double                                     goalBias_;

            /** \brief The maximum length of a motion to be added to a tree */
            double                                     maxDistance_;

            /** \brief The random number generator */
            RNG                                        rng_;
        };

    }
}

#endif
