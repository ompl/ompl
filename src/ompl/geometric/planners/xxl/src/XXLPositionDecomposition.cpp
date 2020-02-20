/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Rice University
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

/* Author: Ryan Luna */

#include <stack>
#include "ompl/geometric/planners/xxl/XXLPositionDecomposition.h"
#include "ompl/util/Exception.h"

ompl::geometric::XXLPositionDecomposition::XXLPositionDecomposition(const base::RealVectorBounds &bounds,
                                                                    const std::vector<int> &slices, bool diagonalEdges)
  : bounds_(bounds), slices_(slices), diagonalEdges_(diagonalEdges)
{
    bounds_.check();

    if (slices_.size() == 0)
        throw ompl::Exception("There must be at least one dimension specified in slices");
    if (bounds_.low.size() != slices_.size())
        throw ompl::Exception("Total slice count must be equal to the number of dimensions");

    numRegions_ = slices_[0];
    for (size_t i = 1; i < slices_.size(); ++i)
        numRegions_ *= slices_[i];

    cellSizes_.resize(slices_.size());
    for (size_t i = 0; i < slices_.size(); ++i)
    {
        double extent = fabs(bounds_.high[i] - bounds_.low[i]);
        cellSizes_[i] = extent / slices_[i];
    }
}

ompl::geometric::XXLPositionDecomposition::~XXLPositionDecomposition()
{
}

int ompl::geometric::XXLPositionDecomposition::getNumRegions() const
{
    return numRegions_;
}

int ompl::geometric::XXLPositionDecomposition::getDimension() const
{
    return (int)slices_.size();
}

int ompl::geometric::XXLPositionDecomposition::locateRegion(const base::State *s) const
{
    std::vector<double> coord;
    project(s, coord);
    return coordToRegion(coord);
}

int ompl::geometric::XXLPositionDecomposition::locateRegion(const std::vector<double> &coord) const
{
    return coordToRegion(coord);
}

void ompl::geometric::XXLPositionDecomposition::getNeighbors(int rid, std::vector<int> &neighbors) const
{
    if (diagonalEdges_)
        getDiagonalNeighbors(rid, neighbors);
    else
        getNonDiagonalNeighbors(rid, neighbors);
}

void ompl::geometric::XXLPositionDecomposition::getNeighborhood(int rid, std::vector<int> &neighborhood) const
{
    getDiagonalNeighbors(rid, neighborhood);
}

double ompl::geometric::XXLPositionDecomposition::distanceHeuristic(int r1, int r2) const
{
    std::vector<int> c1, c2;
    ridToGridCell(r1, c1);
    ridToGridCell(r2, c2);

    // manhattan distance for everything
    double dist = 0.0;
    for (size_t i = 0; i < 2; ++i)
        dist += abs(c1[i] - c2[i]);

    // should do Chebyshev for diagonal edges...

    return dist;
}

void ompl::geometric::XXLPositionDecomposition::ridToGridCell(int rid, std::vector<int> &cell) const
{
    cell.resize(slices_.size());

    int lowerDimensionalRegions = numRegions_ / slices_.back();
    for (int i = slices_.size() - 1; i >= 0; --i)
    {
        cell[i] = rid / lowerDimensionalRegions;

        rid %= lowerDimensionalRegions;
        if (i > 0)
            lowerDimensionalRegions /= slices_[i - 1];
    }
}

int ompl::geometric::XXLPositionDecomposition::gridCellToRid(const std::vector<int> &cell) const
{
    int region = cell[0];
    int lowerDimensionalRegions = slices_[0];
    for (size_t i = 1; i < cell.size(); ++i)
    {
        region += (cell[i] * lowerDimensionalRegions);
        lowerDimensionalRegions *= slices_[i - 1];
    }
    return region;
}

int ompl::geometric::XXLPositionDecomposition::coordToRegion(const std::vector<double> &coord) const
{
    return coordToRegion(&coord[0]);
}

int ompl::geometric::XXLPositionDecomposition::coordToRegion(const double *coord) const
{
    std::vector<int> cell(slices_.size());
    for (size_t i = 0; i < slices_.size(); ++i)
        cell[i] = (coord[i] - bounds_.low[i]) / cellSizes_[i];
    return gridCellToRid(cell);
}

bool ompl::geometric::XXLPositionDecomposition::hasDiagonalEdges() const
{
    return diagonalEdges_;
}

void ompl::geometric::XXLPositionDecomposition::getNonDiagonalNeighbors(int rid, std::vector<int> &neighbors) const
{
    std::vector<int> cell;
    ridToGridCell(rid, cell);
    std::vector<int> workCell(cell.begin(), cell.end());

    for (size_t i = 0; i < slices_.size(); ++i)
    {
        if (slices_[i] == 1)  // no neighbors in this dimension
            continue;

        workCell[i] -= 1;
        if (workCell[i] >= 0 && workCell[i] < slices_[i])
            neighbors.push_back(gridCellToRid(workCell));

        if (slices_[i] > 2)
        {
            workCell[i] += 2;
            if (workCell[i] >= 0 && workCell[i] < slices_[i])
                neighbors.push_back(gridCellToRid(workCell));
        }
        workCell[i] = cell[i];
    }
}

void ompl::geometric::XXLPositionDecomposition::getDiagonalNeighbors(int rid, std::vector<int> &neighbors) const
{
    std::vector<int> ridCell;
    ridToGridCell(rid, ridCell);

    std::stack<std::pair<int, std::vector<int>>> stack;
    stack.push(std::make_pair(0, ridCell));

    while (!stack.empty())
    {
        std::pair<int, std::vector<int>> entry = stack.top();
        stack.pop();

        if (entry.first == (int)slices_.size())
        {
            // make sure we don't add ourself as a neighbor
            bool same = true;
            for (size_t i = 0; i < entry.second.size() && same; ++i)
                if (entry.second[i] != ridCell[i])
                    same = false;

            if (!same)
                neighbors.push_back(gridCellToRid(entry.second));
        }
        else
        {
            int i = entry.first;

            entry.second[i] -= 1;
            if (entry.second[i] >= 0)  // don't go out of bounds
                stack.push(std::make_pair(i + 1, entry.second));

            entry.second[i] += 1;
            stack.push(std::make_pair(i + 1, entry.second));

            entry.second[i] += 1;
            if (entry.second[i] < slices_[i])
                stack.push(std::make_pair(i + 1, entry.second));
        }
    }
}
