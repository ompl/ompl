/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Rice University
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

/* Author: Matt Maly */

#include "ompl/control/planners/syclop/GridDecomposition.h"

namespace
{
    /** \brief Helper method to return len^dim for calculating number of regions. */
    static int calcNumGridCells(int len, int dim);
}

ompl::control::GridDecomposition::GridDecomposition(int len, int dim, const base::RealVectorBounds &b)
  : Decomposition(dim, b), length_(len), cellVolume_(b.getVolume()), numGridCells_(calcNumGridCells(len, dim))
{
    double lenInv = 1.0 / len;
    for (int i = 0; i < dim; ++i)
        cellVolume_ *= lenInv;
}

void ompl::control::GridDecomposition::getNeighbors(int rid, std::vector<int> &neighbors) const
{
    // We efficiently compute neighbors for dim = 1, 2, or 3; for higher dimensions we use a general approach.
    if (dimension_ == 1)
    {
        if (rid > 0)
            neighbors.push_back(rid - 1);
        if (rid < length_ - 1)
            neighbors.push_back(rid + 1);
    }
    else if (dimension_ == 2)
    {
        static const int offset[] = {-1, -1, 0, -1, +1, -1, -1, 0, +1, 0, -1, +1, 0, +1, +1, +1};
        std::vector<int> coord(2);
        regionToGridCoord(rid, coord);
        std::vector<int> nc(2);
        for (std::size_t i = 0; i < 16; i += 2)
        {
            nc[0] = coord[0] + offset[i];
            nc[1] = coord[1] + offset[i + 1];
            if (nc[0] >= 0 && (int)nc[0] < length_ && nc[1] >= 0 && (int)nc[1] < length_)
                neighbors.push_back(nc[0] * length_ + nc[1]);
        }
    }
    else if (dimension_ == 3)
    {
        static const int offset[] = {-1, 0,  0,  +1, 0,  0,  0,  -1, 0,  0,  +1, 0,  -1, -1, 0,  -1, +1, 0,  +1, -1,
                                     0,  +1, +1, 0,  -1, 0,  -1, +1, 0,  -1, 0,  -1, -1, 0,  +1, -1, -1, -1, -1, -1,
                                     +1, -1, +1, -1, -1, +1, +1, -1, -1, 0,  +1, +1, 0,  +1, 0,  -1, +1, 0,  +1, +1,
                                     -1, -1, +1, -1, +1, +1, +1, -1, +1, +1, +1, +1, 0,  0,  -1, 0,  0,  +1};
        std::vector<int> coord(3);
        regionToGridCoord(rid, coord);
        std::vector<int> nc(3);
        for (int i = 0; i < 78; i += 3)
        {
            nc[0] = coord[0] + offset[i];
            nc[1] = coord[1] + offset[i + 1];
            nc[2] = coord[2] + offset[i + 2];
            if (nc[0] >= 0 && (int)nc[0] < length_ && nc[1] >= 0 && (int)nc[1] < length_ && nc[2] >= 0 &&
                (int)nc[2] < length_)
                neighbors.push_back(nc[0] * length_ * length_ + nc[1] * length_ + nc[2]);
        }
    }
    else
    {
        computeGridNeighbors(rid, neighbors);
    }
}

int ompl::control::GridDecomposition::locateRegion(const base::State *s) const
{
    std::vector<double> coord(dimension_);
    project(s, coord);
    return coordToRegion(coord);
}

void ompl::control::GridDecomposition::sampleFromRegion(int rid, RNG &rng, std::vector<double> &coord) const
{
    coord.resize(dimension_);
    const base::RealVectorBounds &regionBounds(getRegionBounds(rid));
    for (int i = 0; i < dimension_; ++i)
        coord[i] = rng.uniformReal(regionBounds.low[i], regionBounds.high[i]);
}

void ompl::control::GridDecomposition::computeGridNeighbors(int rid, std::vector<int> &neighbors) const
{
    std::vector<int> candidate(dimension_, -1);
    std::vector<int> coord;
    regionToGridCoord(rid, coord);

    computeGridNeighborsSub(coord, neighbors, 0, candidate);
}

void ompl::control::GridDecomposition::computeGridNeighborsSub(const std::vector<int> &coord,
                                                               std::vector<int> &neighbors, int dim,
                                                               std::vector<int> &candidate) const
{
    // Stopping condition for recursive method.
    if (dim == dimension_)
    {
        // Make sure we don't push back ourselves as a neighbor
        bool same = true;
        for (unsigned int i = 0; i < coord.size() && same; ++i)
            same = (coord[i] == candidate[i]);

        if (!same)
        {
            neighbors.push_back(gridCoordToRegion(candidate));
        }
    }
    else
    {
        // Check neighbor in the cell preceding this one in this dimension
        if (coord[dim] >= 1)
        {
            candidate[dim] = coord[dim] - 1;
            computeGridNeighborsSub(coord, neighbors, dim + 1, candidate);
        }

        // Make sure to include the same coordinate, for neighbors "above", "below", "in front of", "behind", etcetera.
        candidate[dim] = coord[dim];
        computeGridNeighborsSub(coord, neighbors, dim + 1, candidate);

        // Check neighbor in the cell after this one in this dimension
        if (coord[dim] + 1 < length_)
        {
            candidate[dim] = coord[dim] + 1;
            computeGridNeighborsSub(coord, neighbors, dim + 1, candidate);
        }
    }
}

void ompl::control::GridDecomposition::regionToGridCoord(int rid, std::vector<int> &coord) const
{
    coord.resize(dimension_);
    for (int i = dimension_ - 1; i >= 0; --i)
    {
        int remainder = rid % length_;
        coord[i] = remainder;
        rid /= length_;
    }
}

int ompl::control::GridDecomposition::gridCoordToRegion(const std::vector<int> &coord) const
{
    int region = 0;
    for (unsigned int i = 0; i < coord.size(); i++)
    {
        // Computing length_^(dimension of coord -1)
        int multiplicand = 1;
        for (unsigned int j = 1; j < coord.size() - i; j++)
            multiplicand *= length_;

        region += (coord[i] * multiplicand);
    }
    return region;
}

int ompl::control::GridDecomposition::coordToRegion(const std::vector<double> &coord) const
{
    int region = 0;
    int factor = 1;
    int index;
    for (int i = dimension_ - 1; i >= 0; --i)
    {
        index = (int)(length_ * (coord[i] - bounds_.low[i]) / (bounds_.high[i] - bounds_.low[i]));

        // There is an edge case when the coordinate lies exactly on the upper bound where
        // the region index will be out of bounds.  Ensure index lies within [0, length_)
        if (index >= length_)
            index = length_ - 1;

        region += factor * index;
        factor *= length_;
    }
    return region;
}

void ompl::control::GridDecomposition::coordToGridCoord(const std::vector<double> &coord,
                                                        std::vector<int> &gridCoord) const
{
    gridCoord.resize(dimension_);
    for (int i = 0; i < dimension_; ++i)
    {
        gridCoord[i] = (int)(length_ * (coord[i] - bounds_.low[i]) / (bounds_.high[i] - bounds_.low[i]));

        // There is an edge case when the coordinate lies exactly on the upper bound where
        // the region index will be out of bounds.  Ensure index lies within [0, length_)
        if (gridCoord[i] >= length_)
            gridCoord[i] = length_ - 1;
    }
}

const ompl::base::RealVectorBounds &ompl::control::GridDecomposition::getRegionBounds(int rid) const
{
    if (regToBounds_.count(rid) > 0)
        return *regToBounds_[rid];
    auto regionBounds(std::make_shared<ompl::base::RealVectorBounds>(dimension_));
    std::vector<int> rc(dimension_);
    regionToGridCoord(rid, rc);
    for (int i = 0; i < dimension_; ++i)
    {
        const double length = (bounds_.high[i] - bounds_.low[i]) / length_;
        regionBounds->low[i] = bounds_.low[i] + length * rc[i];
        regionBounds->high[i] = regionBounds->low[i] + length;
    }
    regToBounds_[rid] = regionBounds;
    return *regToBounds_[rid];
}

namespace
{
    int calcNumGridCells(int len, int dim)
    {
        int numCells = 1;
        for (int i = 0; i < dim; ++i)
            numCells *= len;
        return numCells;
    }
}
