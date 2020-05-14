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

#include "ompl/geometric/planners/xxl/XXLPlanarDecomposition.h"
#include "ompl/util/Exception.h"
#include "ompl/util/Console.h"
#include <limits>

ompl::geometric::XXLPlanarDecomposition::XXLPlanarDecomposition(const base::RealVectorBounds &xyBounds,
                                                                const std::vector<int> &xySlices, const int thetaSlices,
                                                                bool diagonalEdges)
  : diagonalEdges_(diagonalEdges), xyBounds_(xyBounds), xySlices_(xySlices), thetaSlices_(thetaSlices)
{
    if (xySlices_.size() != 2)
        throw ompl::Exception("%s: xySlices must have length 2", __func__);
    if (thetaSlices_ < 1)
        throw ompl::Exception("%s: thetaSlices must be at least 1", __func__);
    xyBounds_.check();

    if (thetaLow_ > thetaHigh_)
        throw ompl::Exception("%s: theta lower bound > theta upper bound");

    numRegions_ = 1;
    for (size_t i = 0; i < xySlices_.size(); ++i)
    {
        if (xySlices_[i] < 1)
            throw ompl::Exception("%s: Number of xySlices must be positive", __func__);
        numRegions_ *= xySlices_[i];
    }
    numRegions_ *= thetaSlices_;

    // region volume will be the position part only...
    dx_ = std::abs(xyBounds.high[0] - xyBounds.low[0]);
    dy_ = std::abs(xyBounds.high[1] - xyBounds.low[1]);
    dTheta_ = std::abs(thetaHigh_ - thetaLow_);

    // The size of each grid cell in the XYZ dimensions
    xSize_ = dx_ / xySlices_[0];
    ySize_ = dy_ / xySlices_[1];
    thetaSize_ = dTheta_ / thetaSlices_;

    dimension_ = 1;
    if (xySlices_[0] > 1 || xySlices_[1] > 1)
        dimension_++;
    if (thetaSlices_ > 1)
        dimension_++;

    assert(dimension_ >= 1 && dimension_ <= 3);
}

ompl::geometric::XXLPlanarDecomposition::XXLPlanarDecomposition(const base::RealVectorBounds &xyBounds,
                                                                const std::vector<int> &xySlices, const int thetaSlices,
                                                                double thetaLowerBound, double thetaUpperBound,
                                                                bool diagonalEdges)
  : XXLDecomposition()
  , diagonalEdges_(diagonalEdges)
  , xyBounds_(xyBounds)
  , thetaLow_(thetaLowerBound)
  , thetaHigh_(thetaUpperBound)
  , xySlices_(xySlices)
  , thetaSlices_(thetaSlices)
{
    if (xySlices_.size() != 2)
        throw Exception("%s: xySlices must have length 2", __func__);
    if (thetaSlices_ < 1)
        throw Exception("%s: thetaSlices must be at least 1", __func__);
    xyBounds_.check();

    if (thetaLow_ > thetaHigh_)
        throw Exception("%s: theta lower bound > theta upper bound");

    numRegions_ = 1;
    for (size_t i = 0; i < xySlices_.size(); ++i)
    {
        if (xySlices_[i] < 1)
            throw Exception("%s: Number of xySlices must be positive", __func__);
        numRegions_ *= xySlices_[i];
    }
    numRegions_ *= thetaSlices_;

    // region volume will be the position part only...
    dx_ = fabs(xyBounds.high[0] - xyBounds.low[0]);
    dy_ = fabs(xyBounds.high[1] - xyBounds.low[1]);
    dTheta_ = fabs(thetaHigh_ - thetaLow_);

    // The size of each grid cell in the XYZ dimensions
    xSize_ = dx_ / xySlices_[0];
    ySize_ = dy_ / xySlices_[1];
    thetaSize_ = dTheta_ / thetaSlices_;

    dimension_ = 1;
    if (xySlices_[0] > 1 || xySlices_[1] > 1)
        dimension_++;
    if (thetaSlices_ > 1)
        dimension_++;

    assert(dimension_ >= 1 && dimension_ <= 3);
}

ompl::geometric::XXLPlanarDecomposition::~XXLPlanarDecomposition()
{
}

int ompl::geometric::XXLPlanarDecomposition::getNumRegions() const
{
    return numRegions_;
}

int ompl::geometric::XXLPlanarDecomposition::getDimension() const
{
    return dimension_;
}

int ompl::geometric::XXLPlanarDecomposition::locateRegion(const base::State *s) const
{
    std::vector<double> coord;
    project(s, coord);
    return coordToRegion(coord);
}

int ompl::geometric::XXLPlanarDecomposition::locateRegion(const std::vector<double> &coord) const
{
    return coordToRegion(coord);
}

void ompl::geometric::XXLPlanarDecomposition::getNeighbors(int rid, std::vector<int> &neighbors) const
{
    // up, down, left, right for position dimensions
    // same for orientation, but we must handle the wrap around case carefully
    if (diagonalEdges_)
        getDiagonalNeighbors(rid, neighbors);
    else
        getNonDiagonalNeighbors(rid, neighbors);
}

void ompl::geometric::XXLPlanarDecomposition::getNeighborhood(int rid, std::vector<int> &neighborhood) const
{
    getDiagonalNeighbors(rid, neighborhood);
}

void ompl::geometric::XXLPlanarDecomposition::getNonDiagonalNeighbors(int rid, std::vector<int> &neighbors) const
{
    std::vector<int> ridCell;
    ridToGridCell(rid, ridCell);

    std::vector<int> cell(ridCell.begin(), ridCell.end());
    std::vector<int> workCell(ridCell.begin(), ridCell.end());

    // xy
    for (size_t i = 0; i < 2; ++i)
    {
        // There are no neighbors in this dimension
        if (xySlices_[i] == 1)
            continue;

        workCell[i] -= 1;
        if (workCell[i] >= 0 && workCell[i] < xySlices_[i])
            neighbors.push_back(gridCellToRid(workCell));

        if (xySlices_[i] > 2)
        {
            workCell[i] += 2;
            if (workCell[i] >= 0 && workCell[i] < xySlices_[i])
                neighbors.push_back(gridCellToRid(workCell));
            workCell[i] = cell[i];
        }
    }

    // theta
    if (thetaSlices_ > 1)
    {
        workCell[2] -= 1;
        if (workCell[2] < 0)
            workCell[2] += thetaSlices_;
        else if (workCell[2] >= thetaSlices_)
            workCell[2] -= thetaSlices_;
        neighbors.push_back(gridCellToRid(workCell));

        if (thetaSlices_ > 2)
        {
            workCell[2] += 2;
            if (workCell[2] < 0)
                workCell[2] += thetaSlices_;
            else if (workCell[2] >= thetaSlices_)
                workCell[2] -= thetaSlices_;
            neighbors.push_back(gridCellToRid(workCell));
        }
    }
}

void ompl::geometric::XXLPlanarDecomposition::getDiagonalNeighbors(int rid, std::vector<int> &neighbors) const
{
    std::vector<int> ridCell;
    ridToGridCell(rid, ridCell);

    std::vector<int> cell(ridCell.begin(), ridCell.end());

    for (int x = -1; x <= 1; ++x)
    {
        int tX = ridCell[0] + x;
        if (tX >= 0 && tX < xySlices_[0])
            cell[0] = tX;
        else
            continue;

        for (int y = -1; y <= 1; ++y)
        {
            int tY = ridCell[1] + y;
            if (tY >= 0 && tY < xySlices_[1])
                cell[1] = tY;
            else
                continue;

            for (int theta = -1; theta <= 1; ++theta)
            {
                // No additional neighbors in this dimension
                if (thetaSlices_ == 1 && theta != 0)
                    continue;

                // Do not add duplicate neighbors on the wrap around
                if (thetaSlices_ <= 2 && theta > 0)
                    continue;

                // don't add ourself as a neighbor
                if (x == 0 && y == 0 && theta == 0)
                    continue;

                int tTheta = ridCell[2] + theta;
                if (tTheta < 0)
                    tTheta += thetaSlices_;
                else if (tTheta >= thetaSlices_)
                    tTheta -= thetaSlices_;
                cell[2] = tTheta;

                neighbors.push_back(gridCellToRid(cell));
            }
        }
    }
}

int ompl::geometric::XXLPlanarDecomposition::coordToRegion(const std::vector<double> &coord) const
{
    return coordToRegion(&coord[0]);
}

int ompl::geometric::XXLPlanarDecomposition::coordToRegion(const double *coord) const
{
    // must perform computation about origin
    std::vector<int> cell(3);
    cell[0] = (coord[0] - xyBounds_.low[0]) / xSize_;  // x
    cell[1] = (coord[1] - xyBounds_.low[1]) / ySize_;  // y
    cell[2] = (coord[2] - thetaLow_) / thetaSize_;     // theta

    return gridCellToRid(cell);
}

void ompl::geometric::XXLPlanarDecomposition::ridToGridCell(int rid, std::vector<int> &cell) const
{
    cell.resize(3);
    cell[2] = rid / (xySlices_[0] * xySlices_[1]);

    rid %= (xySlices_[0] * xySlices_[1]);
    cell[1] = rid / xySlices_[0];

    rid %= xySlices_[0];  // mod should not be necessary
    cell[0] = rid;
}

int ompl::geometric::XXLPlanarDecomposition::gridCellToRid(const std::vector<int> &cell) const
{
    int region = cell[0];
    region += cell[1] * xySlices_[0];
    region += cell[2] * xySlices_[0] * xySlices_[1];

    return region;
}

double ompl::geometric::XXLPlanarDecomposition::distanceHeuristic(int r1, int r2) const
{
    std::vector<int> c1, c2;
    ridToGridCell(r1, c1);
    ridToGridCell(r2, c2);

    // manhattan distance for everything
    double dist = 0.0;
    for (size_t i = 0; i < 2; ++i)
        dist += std::abs(c1[i] - c2[i]);

    // theta wraps around
    if (thetaSlices_ > 1)
    {
        int min = std::min(c1[2], c2[2]);
        int max = std::max(c1[2], c2[2]);
        dist += std::min(std::abs(c1[2] - c2[2]), std::abs((min + thetaSlices_) - max));
    }

    return dist;
}

bool ompl::geometric::XXLPlanarDecomposition::hasDiagonalEdges() const
{
    return diagonalEdges_;
}

// Sample a point in the SE(2) decomposition (position and orientation)
void ompl::geometric::XXLPlanarDecomposition::sampleCoordinateFromRegion(int r, std::vector<double> &coord) const
{
    coord.resize(3);
    sampleCoordinateFromRegion(r, &coord[0]);
}

void ompl::geometric::XXLPlanarDecomposition::sampleCoordinateFromRegion(int r, double *coord) const
{
    std::vector<int> cell;
    ridToGridCell(r, cell);

    // x
    double xlow = xyBounds_.low[0] + (cell[0] * xSize_);
    coord[0] = rng_.uniformReal(xlow, xlow + xSize_);

    // y
    double ylow = xyBounds_.low[1] + (cell[1] * ySize_);
    coord[1] = rng_.uniformReal(ylow, ylow + ySize_);

    // theta
    double tlow = thetaLow_ + (cell[2] * thetaSize_);
    coord[2] = rng_.uniformReal(tlow, tlow + thetaSize_);
}