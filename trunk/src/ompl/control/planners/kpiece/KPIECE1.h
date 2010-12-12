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

#ifndef OMPL_CONTROL_PLANNERS_KPIECE_KPIECE1_
#define OMPL_CONTROL_PLANNERS_KPIECE_KPIECE1_

#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/datastructures/GridB.h"
#include <vector>

namespace ompl
{

    namespace control
    {

        /**
           @anchor cKPIECE1

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
           This implementation is intended for systems with differential constraints.

           @par External documentation
           I.A. Şucan and L.E. Kavraki, Kinodynamic motion planning by interior-exterior cell exploration,
           in <em>Workshop on the Algorithmic Foundations of Robotics</em>, Dec. 2008.<br>
           <a href="http://ioan.sucan.ro/files/pubs/wafr2008.pdf">[PDF]</a>
        */

        /** \brief Kinodynamic Planning by Interior-Exterior Cell Exploration */
        class KPIECE1 : public base::Planner
        {
        public:

            KPIECE1(const SpaceInformationPtr &si) : base::Planner(si, "KPIECE1")
            {
                type_ = base::PLAN_TO_GOAL_ANY;

                siC_ = si.get();
                goalBias_ = 0.05;
                selectBorderFraction_ = 0.7;
                badScoreFactor_ = 0.3;
                goodScoreFactor_ = 0.9;
                tree_.grid.onCellUpdate(computeImportance, NULL);
            }

            virtual ~KPIECE1(void)
            {
                freeMemory();
            }

            virtual bool solve(const base::PlannerTerminationCondition &ptc);

            virtual void clear(void);

            /** In the process of randomly selecting states in the state
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

            /** Get the goal bias the planner is using */
            double getGoalBias(void) const
            {
                return goalBias_;
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

            class Motion
            {
            public:

                Motion(void) : state(NULL), control(NULL), steps(0), parent(NULL)
                {
                }

                Motion(const SpaceInformation *si) : state(si->allocState()), control(si->allocControl()), steps(0), parent(NULL)
                {
                }

                ~Motion(void)
                {
                }

                base::State       *state;
                Control           *control;
                unsigned int       steps;
                Motion            *parent;

            };

            struct CellData
            {
                CellData(void) : coverage(0.0), selections(1), score(1.0), iteration(0), importance(0.0)
                {
                }

                ~CellData(void)
                {
                }

                std::vector<Motion*> motions;
                double               coverage;
                unsigned int         selections;
                double               score;
                unsigned int         iteration;
                double               importance;
            };

            struct OrderCellsByImportance
            {
                bool operator()(const CellData * const a, const CellData * const b) const
                {
                    return a->importance > b->importance;
                }
            };

            typedef GridB<CellData*, OrderCellsByImportance> Grid;

            struct TreeData
            {
                TreeData(void) : grid(0), size(0), iteration(1)
                {
                }

                Grid         grid;
                unsigned int size;
                unsigned int iteration;
            };

            static void computeImportance(Grid::Cell *cell, void*)
            {
                CellData &cd = *(cell->data);
                cd.importance =  cd.score / ((cell->neighbors + 1) * cd.coverage * cd.selections);
            }

            void freeMemory(void);
            void freeGridMotions(Grid &grid);
            void freeCellData(CellData *cdata);
            void freeMotion(Motion *motion);

            unsigned int addMotion(Motion* motion, double dist);
            bool selectMotion(Motion* &smotion, Grid::Cell* &scell);
            unsigned int findNextMotion(const Grid::Coord &origin, const std::vector<Grid::Coord> &coords, unsigned int index, unsigned int last);

            ControlSamplerPtr             controlSampler_;

            TreeData                      tree_;

            const SpaceInformation       *siC_;

            base::ProjectionEvaluatorPtr  projectionEvaluator_;

            double                        goodScoreFactor_;
            double                        badScoreFactor_;
            double                        selectBorderFraction_;
            double                        goalBias_;
            RNG                           rng_;
        };

    }
}

#endif
