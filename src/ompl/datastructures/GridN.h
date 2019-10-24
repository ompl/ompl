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

#ifndef OMPL_DATASTRUCTURES_GRID_N_
#define OMPL_DATASTRUCTURES_GRID_N_

#include "ompl/datastructures/Grid.h"

namespace ompl
{
    /** \brief Representation of a grid where cells keep track of how many neighbors they have */
    template <typename _T>
    class GridN : public Grid<_T>
    {
    public:
        /// Datatype for cell in base class
        using BaseCell = typename Grid<_T>::Cell;

        /// Datatype for array of cells in base class
        using BaseCellArray = typename Grid<_T>::CellArray;

        /// Datatype for cell coordinates
        using Coord = typename Grid<_T>::Coord;

        /// Definition of a cell in this grid
        struct Cell : public BaseCell
        {
            /// The number of neighbors
            unsigned int neighbors{0};

            /// A flag indicating whether this cell is on the border or not
            bool border{true};

            Cell() = default;

            ~Cell() override = default;

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

        /// The datatype for arrays of cells
        using CellArray = std::vector<Cell *>;

        /// The constructor takes the dimension of the grid as argument
        explicit GridN(unsigned int dimension) : Grid<_T>(dimension)
        {
            hasBounds_ = false;
            overrideCellNeighborsLimit_ = false;
            setDimension(dimension);
        }

        ~GridN() override = default;

        /// Update the dimension of the grid; this should not be done
        /// unless the grid is empty
        void setDimension(unsigned int dimension)
        {
            assert(Grid<_T>::empty() == true);
            Grid<_T>::dimension_ = dimension;
            Grid<_T>::maxNeighbors_ = 2 * dimension;
            if (!overrideCellNeighborsLimit_)
                interiorCellNeighborsLimit_ = Grid<_T>::maxNeighbors_;
        }

        /// If bounds for the grid need to be considered, we can set them here.
        /// When the number of neighbors are counted, whether the
        /// Space is bounded matters, in the sense that if a cell is on
        /// the boundary, we know some of its neighbors cannot exist.
        /// In order to allow such a cell to reflect the fact it has
        /// Achieved its maximal number of neighbors, the boundary is
        /// counted as the number of neighbors it prevents from
        /// existing.
        void setBounds(const Coord &low, const Coord &up)
        {
            lowBound_ = low;
            upBound_ = up;
            hasBounds_ = true;
        }

        /// Set the limit of neighboring cells to determine when a cell becomes interior
        /// by default, this is 2 * dimension of grid
        void setInteriorCellNeighborLimit(unsigned int count)
        {
            interiorCellNeighborsLimit_ = count;
            assert(interiorCellNeighborsLimit_ > 0);
            overrideCellNeighborsLimit_ = true;
        }

        /// Get the cell at a specified coordinate
        Cell *getCell(const Coord &coord) const
        {
            return static_cast<Cell *>(Grid<_T>::getCell(coord));
        }

        /// Get the list of neighbors for a given cell
        void neighbors(const Cell *cell, CellArray &list) const
        {
            Coord test = cell->coord;
            neighbors(test, list);
        }

        /// Get the list of neighbors for a given coordinate
        void neighbors(const Coord &coord, CellArray &list) const
        {
            Coord test = coord;
            neighbors(test, list);
        }

        /// Get the list of neighbors for a given coordinate
        void neighbors(Coord &coord, CellArray &list) const
        {
            BaseCellArray baselist;
            Grid<_T>::neighbors(coord, baselist);
            list.reserve(list.size() + baselist.size());
            for (const auto &c : baselist)
                list.push_back(static_cast<Cell *>(c));
        }

        /// Instantiate a new cell at given coordinates;
        /// Optionally return the list of future neighbors.  Note:
        /// this call only creates the cell, but does not add it to
        /// the grid.  It however updates the neighbor count for
        /// neighboring cells
        BaseCell *createCell(const Coord &coord, BaseCellArray *nbh = nullptr) override
        {
            auto *cell = new Cell();
            cell->coord = coord;

            BaseCellArray *list = nbh ? nbh : new BaseCellArray();
            Grid<_T>::neighbors(cell->coord, *list);

            for (auto cl = list->begin(); cl != list->end(); ++cl)
            {
                auto *c = static_cast<Cell *>(*cl);
                c->neighbors++;
                if (c->border && c->neighbors >= interiorCellNeighborsLimit_)
                    c->border = false;
            }

            cell->neighbors = numberOfBoundaryDimensions(cell->coord) + list->size();
            if (cell->border && cell->neighbors >= interiorCellNeighborsLimit_)
                cell->border = false;

            if (!nbh)
                delete list;

            return cell;
        }

        /// Remove a cell from the grid. If the cell has not been
        /// Added to the grid, only update the neighbor list
        bool remove(BaseCell *cell) override
        {
            if (cell)
            {
                auto *list = new BaseCellArray();
                Grid<_T>::neighbors(cell->coord, *list);
                for (auto cl = list->begin(); cl != list->end(); ++cl)
                {
                    auto *c = static_cast<Cell *>(*cl);
                    c->neighbors--;
                    if (!c->border && c->neighbors < interiorCellNeighborsLimit_)
                        c->border = true;
                }
                delete list;
                auto pos = Grid<_T>::hash_.find(&cell->coord);
                if (pos != Grid<_T>::hash_.end())
                {
                    Grid<_T>::hash_.erase(pos);
                    return true;
                }
            }
            return false;
        }

        /// Get the set of instantiated cells in the grid
        void getCells(CellArray &cells) const
        {
            for (const auto &h : Grid<_T>::hash_)
                cells.push_back(static_cast<Cell *>(h.second));
        }

    protected:
        /// Compute how many sides of a coordinate touch the boundaries of the grid
        unsigned int numberOfBoundaryDimensions(const Coord &coord) const
        {
            unsigned int result = 0;
            if (hasBounds_)
            {
                for (unsigned int i = 0; i < Grid<_T>::dimension_; ++i)
                    if (coord[i] == lowBound_[i] || coord[i] == upBound_[i])
                        result++;
            }
            return result;
        }

        /// Flag indicating whether bounds are in effect for this grid
        bool hasBounds_;

        /// If bounds are set, this defines the lower corner cell
        Coord lowBound_;

        /// If bounds are set, this defines the upper corner cell
        Coord upBound_;

        /// By default, cells are considered on the border if 2n
        /// neighbors are created, for a space of dimension n.
        /// this value is overridden and set in this member variable
        unsigned int interiorCellNeighborsLimit_;

        /// Flag indicating whether the neighbor count used to determine whether
        /// a cell is on the border or not
        bool overrideCellNeighborsLimit_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

#endif
