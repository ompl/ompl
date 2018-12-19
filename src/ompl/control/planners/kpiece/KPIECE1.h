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
#include <set>

namespace ompl
{
    namespace control
    {
        /**
           @anchor cKPIECE1
           @par Short description
           KPIECE is a tree-based planner that uses a discretization
           (multiple levels, in general) to guide the exploration of
           the continuous space. This implementation is a simplified
           one, using a single level of discretization: one grid. The
           grid is imposed on a projection of the state space. When
           exploring the space, preference is given to the boundary of
           this grid. The boundary is computed to be the set of grid
           cells that have less than 2n non-diagonal neighbors in an
           n-dimensional projection space.
           It is important to set the projection the algorithm uses
           (setProjectionEvaluator() function). If no projection is
           set, the planner will attempt to use the default projection
           associated to the state space. An exception is thrown if
           no default projection is available either.
           This implementation is intended for systems with differential constraints.
           @par External documentation
           I.A. Şucan and L.E. Kavraki, Kinodynamic motion planning by interior-exterior cell exploration,
           in <em>Workshop on the Algorithmic Foundations of Robotics</em>, Dec. 2008.<br>
           [[PDF]](http://ioan.sucan.ro/files/pubs/wafr2008.pdf)
        */

        /** \brief Kinodynamic Planning by Interior-Exterior Cell Exploration */
        class KPIECE1 : public base::Planner
        {
        public:
            /** \brief Constructor */
            KPIECE1(const SpaceInformationPtr &si);

            ~KPIECE1() override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void clear() override;

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
            double getGoalBias() const
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
            double getBorderFraction() const
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
                setGoodCellScoreFactor(good);
                setBadCellScoreFactor(bad);
            }

            /** \brief Set the factor that is to be applied to a cell's score when an expansion from that cell fails */
            void setBadCellScoreFactor(double bad)
            {
                badScoreFactor_ = bad;
            }

            /** \brief Set the factor that is to be applied to a cell's score when an expansion from that cell succeedes
             */
            void setGoodCellScoreFactor(double good)
            {
                goodScoreFactor_ = good;
            }

            /** \brief Get the factor that is multiplied to a cell's
                score if extending a motion from that cell succeeded. */
            double getGoodCellScoreFactor() const
            {
                return goodScoreFactor_;
            }

            /** \brief Get the factor that is multiplied to a cell's
                score if extending a motion from that cell failed. */
            double getBadCellScoreFactor() const
            {
                return badScoreFactor_;
            }

            /** \brief When motions reach close to the goal, they are stored in a separate queue
                to allow biasing towards the goal. This function sets the maximum size of that queue. */
            void setMaxCloseSamplesCount(unsigned int nCloseSamples)
            {
                nCloseSamples_ = nCloseSamples;
            }

            /** \brief Get the maximum number of samples to store in the queue of samples that are close to the goal */
            unsigned int getMaxCloseSamplesCount() const
            {
                return nCloseSamples_;
            }

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

            /** \brief Get the projection evaluator */
            const base::ProjectionEvaluatorPtr &getProjectionEvaluator() const
            {
                return projectionEvaluator_;
            }

            void setup() override;
            void getPlannerData(base::PlannerData &data) const override;

        protected:
            /** \brief Representation of a motion for this algorithm */
            struct Motion
            {
                Motion() = default;

                /** \brief Constructor that allocates memory for the state and the control */
                Motion(const SpaceInformation *si)
                  : state(si->allocState()), control(si->allocControl())
                {
                }

                ~Motion() = default;

                /** \brief The state contained by this motion */
                base::State *state{nullptr};

                /** \brief The control contained by this motion */
                Control *control{nullptr};

                /** \brief The number of steps the control is applied for */
                unsigned int steps{0};

                /** \brief The parent motion in the exploration tree */
                Motion *parent{nullptr};
            };

            /** \brief The data held by a cell in the grid of motions */
            struct CellData
            {
                CellData() = default;

                ~CellData() = default;

                /** \brief The set of motions contained in this grid cell */
                std::vector<Motion *> motions;

                /** \brief A measure of coverage for this cell. For
                    this implementation, this is the sum of motion
                    durations */
                double coverage{0.0};

                /** \brief The number of times this cell has been
                    selected for expansion */
                unsigned int selections{1};

                /** \brief A heuristic score computed based on
                    distance to goal (if available), successes and
                    failures at expanding from this cell. */
                double score{1.0};

                /** \brief The iteration at which this cell was created */
                unsigned int iteration{0};

                /** \brief The computed importance (based on other class members) */
                double importance{0.0};
            };

            /** \brief Definintion of an operator passed to the Grid
                structure, to order cells by importance */
            struct OrderCellsByImportance
            {
                bool operator()(const CellData *const a, const CellData *const b) const
                {
                    return a->importance > b->importance;
                }
            };

            /** \brief The datatype for the maintained grid datastructure */
            using Grid = GridB<CellData *, OrderCellsByImportance>;

            /** \brief Information about a known good sample (closer to the goal than others) */
            struct CloseSample
            {
                /** \brief Constructor fully initializes the content of this structure */
                CloseSample(Grid::Cell *c, Motion *m, double d) : cell(c), motion(m), distance(d)
                {
                }

                /** \brief The cell of the motion that is close to the goal */
                Grid::Cell *cell;

                /** \brief The motion that is close to the goal */
                Motion *motion;

                /** \brief The distance to the goal. This value is increased over time, as the number of selections for
                 * this sample increases */
                double distance;

                /** \brief Sort samples in accordance to their distance to the goal */
                bool operator<(const CloseSample &other) const
                {
                    return distance < other.distance;
                }
            };

            /** \brief Bounded set of good samples */
            struct CloseSamples
            {
                /** \brief Construct an object to maintain a set of at most \e size samples */
                CloseSamples(unsigned int size) : maxSize(size)
                {
                }

                /** \brief Evaluate whether motion \e motion, part of
                    cell \e cell is good enough to be part of the set
                    of samples closest to the goal, given its distance
                    to the goal is \e distance. If so, add it to the
                    set and return true. Otherwise, return false.*/
                bool consider(Grid::Cell *cell, Motion *motion, double distance);

                /** \brief Select the top sample (closest to the goal)
                    and update its position in the set subsequently
                    (pretend the distance to the goal is
                    larger). Returns true if the sample selection is
                    successful. */
                bool selectMotion(Motion *&smotion, Grid::Cell *&scell);

                /** \brief Return true if samples can be selected from this set */
                bool canSample() const
                {
                    return !samples.empty();
                }

                /** \brief Maximum number of samples to maintain */
                unsigned int maxSize;

                /** \brief The maintained samples */
                std::set<CloseSample> samples;
            };

            /** \brief The data defining a tree of motions for this algorithm */
            struct TreeData
            {
                TreeData() = default;

                /** \brief A grid containing motions, imposed on a
                    projection of the state space */
                Grid grid{0};

                /** \brief The total number of motions (there can be
                    multiple per cell) in the grid */
                unsigned int size{0};

                /** \brief The number of iterations performed on this tree */
                unsigned int iteration{1};
            };

            /** \brief This function is provided as a calback to the
                grid datastructure to update the importance of a
                cell */
            static void computeImportance(Grid::Cell *cell, void * /*unused*/)
            {
                CellData &cd = *(cell->data);
                cd.importance = cd.score / ((cell->neighbors + 1) * cd.coverage * cd.selections);
            }

            /** \brief Free all the memory allocated by this planner */
            void freeMemory();

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
            Grid::Cell *addMotion(Motion *motion, double dist);

            /** \brief Select a motion and the cell it is part of from
                the grid of motions. This is where preference is given
                to cells on the boundary of the grid.*/
            bool selectMotion(Motion *&smotion, Grid::Cell *&scell);

            /** \brief When generated motions are to be added to the tree of motions, they often need to be split, so
               they don't cross cell boundaries.
                Given that a motion starts out in the cell \e origin and it crosses the cells in \e coords[\e index]
               through \e coords[\e last] (inclusively),
                return the index of the state to be used in the next part of the motion (that is within a cell). This
               will be a value between \e index and \e last. */
            unsigned int findNextMotion(const std::vector<Grid::Coord> &coords, unsigned int index, unsigned int count);

            /** \brief A control sampler */
            ControlSamplerPtr controlSampler_;

            /** \brief The tree datastructure */
            TreeData tree_;

            /** \brief The base::SpaceInformation cast as control::SpaceInformation, for convenience */
            const SpaceInformation *siC_;

            /** \brief This algorithm uses a discretization (a grid)
                to guide the exploration. The exploration is imposed
                on a projection of the state space. */
            base::ProjectionEvaluatorPtr projectionEvaluator_;

            /** \brief When extending a motion from a cell, the
                extension can be successful. If it is, the score of the
                cell is multiplied by this factor. */
            double goodScoreFactor_{0.9};

            /** \brief When extending a motion from a cell, the
                extension can fail. If it is, the score of the cell is
                multiplied by this factor. */
            double badScoreFactor_{0.45};

            /** \brief When motions reach close to the goal, they are stored in a separate queue
                to allow biasing towards the goal. This variable specifies the maximum number of samples
                to keep in that queue. */
            unsigned int nCloseSamples_{30};

            /** \brief The fraction of time to focus exploration on
                the border of the grid. */
            double selectBorderFraction_{0.8};

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is
             * available) */
            double goalBias_{0.05};

            /** \brief The random number generator */
            RNG rng_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion *lastGoalMotion_{nullptr};
        };
    }
}

#endif
