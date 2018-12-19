/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Rice University
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

#ifndef OMPL_GEOMETRIC_PLANNERS_KPIECE_DISCRETIZATION_
#define OMPL_GEOMETRIC_PLANNERS_KPIECE_DISCRETIZATION_

#include "ompl/base/Planner.h"
#include "ompl/datastructures/GridB.h"
#include "ompl/util/Exception.h"
#include <functional>
#include <utility>
#include <vector>
#include <limits>
#include <cassert>
#include <utility>
#include <cstdlib>
#include <cmath>

namespace ompl
{
    namespace geometric
    {
        /** \brief One-level discretization used for KPIECE */
        template <typename Motion>
        class Discretization
        {
        public:
            /** \brief The data held by a cell in the grid of motions */
            struct CellData
            {
                CellData() = default;

                ~CellData() = default;

                /** \brief The set of motions contained in this grid cell */
                std::vector<Motion *> motions;

                /** \brief A measure of coverage for this cell. For
                    this implementation, this is the sum of motion
                    lengths */
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
                /** \brief Order function */
                bool operator()(const CellData *const a, const CellData *const b) const
                {
                    return a->importance > b->importance;
                }
            };

            /** \brief The datatype for the maintained grid datastructure */
            using Grid = GridB<CellData *, OrderCellsByImportance>;

            /** \brief The datatype for the maintained grid cells */
            using Cell = typename Grid::Cell;

            /** \brief The datatype for the maintained grid coordinates */
            using Coord = typename Grid::Coord;

            /** \brief The signature of a function that frees the memory for a motion */
            using FreeMotionFn = typename std::function<void(Motion *)>;

            Discretization(FreeMotionFn freeMotion)
              : grid_(0), size_(0), iteration_(1), recentCell_(nullptr), freeMotion_(std::move(freeMotion))
            {
                grid_.onCellUpdate(computeImportance, nullptr);
                selectBorderFraction_ = 0.9;
            }

            ~Discretization()
            {
                freeMemory();
            }

            /** \brief Set the fraction of time for focusing on the
                border (between 0 and 1). This is the minimum fraction
                used to select cells that are exterior (minimum
                because if 95% of cells are on the border, they will
                be selected with 95% chance, even if this fraction is
                set to 90%)*/
            void setBorderFraction(double bp)
            {
                if (bp < std::numeric_limits<double>::epsilon() || bp > 1.0)
                    throw Exception("The fraction of time spent selecting border cells must be in the range (0,1]");
                selectBorderFraction_ = bp;
            }

            /** \brief Set the fraction of time for focusing on the
                border (between 0 and 1). */
            double getBorderFraction() const
            {
                return selectBorderFraction_;
            }

            /** \brief Set the dimension of the grid to be maintained */
            void setDimension(unsigned int dim)
            {
                grid_.setDimension(dim);
            }

            /** \brief Restore the discretization to its original form */
            void clear()
            {
                freeMemory();
                size_ = 0;
                iteration_ = 1;
                recentCell_ = nullptr;
            }

            void countIteration()
            {
                ++iteration_;
            }

            std::size_t getMotionCount() const
            {
                return size_;
            }

            std::size_t getCellCount() const
            {
                return grid_.size();
            }

            /** \brief Free the memory for the motions contained in a grid */
            void freeMemory()
            {
                for (auto it = grid_.begin(); it != grid_.end(); ++it)
                    freeCellData(it->second->data);
                grid_.clear();
            }

            /** \brief Add a motion to the grid containing motions. As
                a hint, \e dist specifies the distance to the goal
                from the state of the motion being added. The function
                returns the number of cells created to accommodate the
                new motion (0 or 1). The discretization takes
                ownership of the motion passed as argument, and the
                memory for the motion is freed by calling the function
                passed to the constructor. */
            unsigned int addMotion(Motion *motion, const Coord &coord, double dist = 0.0)
            {
                Cell *cell = grid_.getCell(coord);

                unsigned int created = 0;
                if (cell)
                {
                    cell->data->motions.push_back(motion);
                    cell->data->coverage += 1.0;
                    grid_.update(cell);
                }
                else
                {
                    cell = grid_.createCell(coord);
                    cell->data = new CellData();
                    cell->data->motions.push_back(motion);
                    cell->data->coverage = 1.0;
                    cell->data->iteration = iteration_;
                    cell->data->selections = 1;
                    cell->data->score = (1.0 + log((double)(iteration_))) / (1.0 + dist);
                    grid_.add(cell);
                    recentCell_ = cell;
                    created = 1;
                }
                ++size_;
                return created;
            }

            /** \brief Select a motion and the cell it is part of from
                the grid of motions. This is where preference is given
                to cells on the boundary of the grid.*/
            void selectMotion(Motion *&smotion, Cell *&scell)
            {
                scell = rng_.uniform01() < std::max(selectBorderFraction_, grid_.fracExternal()) ? grid_.topExternal() :
                                                                                                   grid_.topInternal();

                // We are running on finite precision, so our update scheme will end up
                // with 0 values for the score. This is where we fix the problem
                if (scell->data->score < std::numeric_limits<double>::epsilon())
                {
                    std::vector<CellData *> content;
                    content.reserve(grid_.size());
                    grid_.getContent(content);
                    for (auto it = content.begin(); it != content.end(); ++it)
                        (*it)->score += 1.0 + log((double)((*it)->iteration));
                    grid_.updateAll();
                }

                assert(scell && !scell->data->motions.empty());

                ++scell->data->selections;
                smotion = scell->data->motions[rng_.halfNormalInt(0, scell->data->motions.size() - 1)];
            }

            bool removeMotion(Motion *motion, const Coord &coord)
            {
                Cell *cell = grid_.getCell(coord);
                if (cell)
                {
                    bool found = false;
                    for (unsigned int i = 0; i < cell->data->motions.size(); ++i)
                        if (cell->data->motions[i] == motion)
                        {
                            cell->data->motions.erase(cell->data->motions.begin() + i);
                            found = true;
                            --size_;
                            break;
                        }
                    if (cell->data->motions.empty())
                    {
                        grid_.remove(cell);
                        freeCellData(cell->data);
                        grid_.destroyCell(cell);
                    }
                    return found;
                }
                return false;
            }

            void updateCell(Cell *cell)
            {
                grid_.update(cell);
            }

            const Grid &getGrid() const
            {
                return grid_;
            }

            void getPlannerData(base::PlannerData &data, int tag, bool start, const Motion *lastGoalMotion) const
            {
                std::vector<CellData *> cdata;
                grid_.getContent(cdata);

                if (lastGoalMotion)
                    data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion->state, tag));

                for (unsigned int i = 0; i < cdata.size(); ++i)
                    for (unsigned int j = 0; j < cdata[i]->motions.size(); ++j)
                    {
                        if (cdata[i]->motions[j]->parent == nullptr)
                        {
                            if (start)
                                data.addStartVertex(base::PlannerDataVertex(cdata[i]->motions[j]->state, tag));
                            else
                                data.addGoalVertex(base::PlannerDataVertex(cdata[i]->motions[j]->state, tag));
                        }
                        else
                        {
                            if (start)
                                data.addEdge(base::PlannerDataVertex(cdata[i]->motions[j]->parent->state, tag),
                                             base::PlannerDataVertex(cdata[i]->motions[j]->state, tag));
                            else
                                data.addEdge(base::PlannerDataVertex(cdata[i]->motions[j]->state, tag),
                                             base::PlannerDataVertex(cdata[i]->motions[j]->parent->state, tag));
                        }
                    }
            }

        private:
            /** \brief Free the memory for the data contained in a grid cell */
            void freeCellData(CellData *cdata)
            {
                for (unsigned int i = 0; i < cdata->motions.size(); ++i)
                    freeMotion_(cdata->motions[i]);
                delete cdata;
            }

            /** \brief This function is provided as a callback to the
                grid datastructure to update the importance of a
                cell */
            static void computeImportance(Cell *cell, void * /*unused*/)
            {
                CellData &cd = *(cell->data);
                cd.importance = cd.score / ((cell->neighbors + 1) * cd.coverage * cd.selections);
            }

            /** \brief A grid containing motions, imposed on a
                projection of the state space */
            Grid grid_;

            /** \brief The total number of motions (there can be
                multiple per cell) in the grid */
            std::size_t size_;

            /** \brief The number of iterations performed on this tree */
            unsigned int iteration_;

            /** \brief The most recently created cell */
            Cell *recentCell_;

            /** \brief Method that can free the memory for a stored motion */
            FreeMotionFn freeMotion_;

            /** \brief The fraction of time to focus exploration on
                the border of the grid. */
            double selectBorderFraction_;

            /** \brief The random number generator */
            RNG rng_;
        };
    }
}

#endif
