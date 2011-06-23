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

#include "ompl/base/StateSpace.h"
#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/util/Exception.h"
#include "ompl/util/RandomNumbers.h"
#include "ompl/tools/config/MagicConstants.h"
#include "ompl/datastructures/Grid.h"
#include <cmath>
#include <cstring>
#include <limits>

static const double DIMENSION_UPDATE_FACTOR = 1.2;

ompl::base::ProjectionMatrix::Matrix ompl::base::ProjectionMatrix::ComputeRandom(const unsigned int from, const unsigned int to, const std::vector<double> &scale)
{
    RNG rng;

    Matrix projection(to);
    for (unsigned int i = 0 ; i < to ; ++i)
    {
        projection[i].resize(from);
        for (unsigned int j = 0 ; j < from ; ++j)
            projection[i][j] = rng.gaussian01();
    }

    for (unsigned int i = 1 ; i < to ; ++i)
    {
        std::valarray<double> &row = projection[i];
        for (unsigned int j = 0 ; j < i ; ++j)
        {
            std::valarray<double> &prevRow = projection[j];
            // subtract projection
            row -= (row * prevRow).sum() * prevRow;
        }
        // normalize
        row /= sqrt((row * row).sum());
    }

    // if we need to apply scaling, do so
    if (scale.size() == from)
    {
        for (unsigned int i = 0 ; i < to ; ++i)
            for (unsigned int j = 0 ; j < from ; ++j)
            {
                if (fabs(scale[i]) < std::numeric_limits<double>::epsilon())
                    throw Exception("Scaling factor must be non-zero");
                projection[i][j] /= scale[i];
            }
    }

    return projection;
}

ompl::base::ProjectionMatrix::Matrix ompl::base::ProjectionMatrix::ComputeRandom(const unsigned int from, const unsigned int to)
{
    return ComputeRandom(from, to, std::vector<double>());
}

void ompl::base::ProjectionMatrix::computeRandom(const unsigned int from, const unsigned int to, const std::vector<double> &scale)
{
    mat = ComputeRandom(from, to, scale);
}

void ompl::base::ProjectionMatrix::computeRandom(const unsigned int from, const unsigned int to)
{
    mat = ComputeRandom(from, to);
}

void ompl::base::ProjectionMatrix::project(const double *from, double *to) const
{
    for (unsigned int i = 0 ; i < mat.size() ; ++i)
    {
        const std::valarray<double> &vec = mat[i];
        const unsigned int dim = vec.size();
        double *pos = to + i;
        *pos = 0.0;
        for (unsigned int j = 0 ; j < dim ; ++j)
            *pos += from[j] * vec[j];
    }
}

void ompl::base::ProjectionMatrix::print(std::ostream &out) const
{
    for (unsigned int i = 0 ; i < mat.size() ; ++i)
    {
        const std::valarray<double> &vec = mat[i];
        const unsigned int dim = vec.size();
        for (unsigned int j = 0 ; j < dim ; ++j)
            out << vec[j] << " ";
        out << std::endl;
    }
}

bool ompl::base::ProjectionEvaluator::userConfigured(void) const
{
    return !defaultCellSizes_ && !cellSizesWereInferred_;
}

void ompl::base::ProjectionEvaluator::setCellSizes(const std::vector<double> &cellSizes)
{
    defaultCellSizes_ = false;
    cellSizesWereInferred_ = false;
    cellSizes_ = cellSizes;
    checkCellSizes();
}

void ompl::base::ProjectionEvaluator::checkCellSizes(void) const
{
    if (getDimension() <= 0)
        throw Exception("Dimension of projection needs to be larger than 0");
    if (cellSizes_.size() != getDimension())
        throw Exception("Number of dimensions in projection space does not match number of cell sizes");
}

void ompl::base::ProjectionEvaluator::defaultCellSizes(void)
{
}

/// @cond IGNORE
namespace ompl
{
    namespace base
    {

        static inline void computeCoordinatesHelper(const std::vector<double> &cellSizes, const EuclideanProjection &projection, ProjectionCoordinates &coord)
        {
            const std::size_t dim = cellSizes.size();
            coord.resize(dim);
            for (unsigned int i = 0 ; i < dim ; ++i)
                coord[i] = (int)floor(projection.values[i]/cellSizes[i]);
        }

        static Grid<int> constructGrid(unsigned int dim, const std::vector<ProjectionCoordinates> &coord)
        {
            Grid<int> g(dim);
            for (std::size_t i = 0 ; i < coord.size() ; ++i)
            {
                Grid<int>::Cell *c = g.getCell(coord[i]);
                if (c)
                    c->data++;
                else
                {
                    Grid<int>::Cell *c = g.createCell(coord[i]);
                    c->data = 1;
                    g.add(c);
                }
            }
            return g;
        }

        static unsigned int getComponentCount(const std::vector<EuclideanProjection*> &proj,
                                              const std::vector<double> &cellSizes)
        {
            std::vector<ProjectionCoordinates> coord(proj.size());
            for (std::size_t i = 0 ; i < proj.size() ; ++i)
                computeCoordinatesHelper(cellSizes, *proj[i], coord[i]);
            return constructGrid(cellSizes.size(), coord).components().size();
        }

        static int updateComponentCountDimension(const std::vector<EuclideanProjection*> &proj,
                                                 std::vector<double> &cellSizes, bool increase)
        {
            unsigned int dim = cellSizes.size();
            const double factor = increase ? DIMENSION_UPDATE_FACTOR : 1.0 / DIMENSION_UPDATE_FACTOR;

            int bestD = -1;
            unsigned int best = 0;
            for (unsigned int d = 0 ; d < dim ; ++d)
            {
                double backup = cellSizes[d];
                cellSizes[d] *= factor;
                unsigned int nc = getComponentCount(proj, cellSizes);
                if (bestD < 0 || (increase && nc > best) || (!increase && nc < best))
                {
                    best = nc;
                    bestD = d;
                }
                cellSizes[d] = backup;
            }
            cellSizes[bestD] *= factor;
            return bestD;
        }

    }
}
/// @endcond




void ompl::base::ProjectionEvaluator::computeCellSizes(const std::vector<const State*> &states)
{
    setup();

    msg_.debug("Computing projections from %u states", states.size());

    unsigned int dim = getDimension();
    std::vector<double> low(dim, std::numeric_limits<double>::infinity());
    std::vector<double> high(dim, -std::numeric_limits<double>::infinity());
    std::vector<EuclideanProjection*>  proj(states.size());
    std::vector<ProjectionCoordinates> coord(states.size());

    for (std::size_t i = 0 ; i < states.size() ; ++i)
    {
        proj[i] = new EuclideanProjection(dim);
        project(states[i], *proj[i]);
        for (std::size_t j = 0 ; j < dim ; ++j)
        {
            if (low[j] > proj[i]->values[j])
                low[j] = proj[i]->values[j];
            if (high[j] < proj[i]->values[j])
                high[j] = proj[i]->values[j];
        }
    }

    bool dir1 = false, dir2 = false;
    do
    {
        for (std::size_t i = 0 ; i < proj.size() ; ++i)
            computeCoordinates(*proj[i], coord[i]);
        const Grid<int> &g = constructGrid(dim, coord);

        const std::vector< std::vector<Grid<int>::Cell*> > &comp = g.components();
        if (comp.size() > 0)
        {
            std::size_t n = comp.size() / 10;
            if (n < 1)
                n = 1;
            std::size_t s = 0;
            for (std::size_t i = 0 ; i < n ; ++i)
                s += comp[i].size();
            double f = (double)s / (double)g.size();

            msg_.debug("There are %u connected components in the projected grid. The first 10%% fractions is %f", comp.size(), f);

            if (f < 0.7)
            {
                dir1 = true;
                int bestD = updateComponentCountDimension(proj, cellSizes_, true);
                msg_.debug("Increasing cell size in dimension %d to %f", bestD, cellSizes_[bestD]);
            }
            else
                if (f > 0.9)
                {
                    dir2 = true;
                    int bestD = updateComponentCountDimension(proj, cellSizes_, false);
                    msg_.debug("Decreasing cell size in dimension %d to %f", bestD, cellSizes_[bestD]);
                }
                else
                {
                    msg_.debug("No more changes made to cell sizes");
                    break;
                }
        }
    } while (dir1 ^ dir2);

    for (unsigned int i = 0 ; i < proj.size() ; ++i)
        delete proj[i];

    // make sure all flags are set correctly
    setCellSizes(cellSizes_);
}

void ompl::base::ProjectionEvaluator::inferCellSizes(void)
{
    cellSizesWereInferred_ = true;
    unsigned int dim = getDimension();
    if (dim > 0)
    {
        StateSamplerPtr sampler = space_->allocStateSampler();
        State *s = space_->allocState();
        EuclideanProjection proj(dim);

        std::vector<double> low(dim, std::numeric_limits<double>::infinity());
        std::vector<double> high(dim, -std::numeric_limits<double>::infinity());

        for (unsigned int i = 0 ; i < magic::PROJECTION_EXTENTS_SAMPLES ; ++i)
        {
            sampler->sampleUniform(s);
            project(s, proj);
            for (unsigned int j = 0 ; j < dim ; ++j)
            {
                if (low[j] > proj.values[j])
                    low[j] = proj.values[j];
                if (high[j] < proj.values[j])
                    high[j] = proj.values[j];
            }
        }

        space_->freeState(s);

        cellSizes_.resize(dim);
        for (unsigned int j = 0 ; j < dim ; ++j)
        {
            cellSizes_[j] = (high[j] - low[j]) / magic::PROJECTION_DIMENSION_SPLITS;
            if (cellSizes_[j] < std::numeric_limits<double>::epsilon())
            {
                cellSizes_[j] = 1.0;
                msg_.warn("Inferred cell size for dimension %u of a projection for state space %s is 0. Setting arbitrary value of 1 instead.",
                          j, space_->getName().c_str());
            }
        }
    }
}

void ompl::base::ProjectionEvaluator::setup(void)
{
    if (defaultCellSizes_)
        defaultCellSizes();

    if ((cellSizes_.size() == 0 && getDimension() > 0) || cellSizesWereInferred_)
        inferCellSizes();

    checkCellSizes();
}

void ompl::base::ProjectionEvaluator::computeCoordinates(const EuclideanProjection &projection, ProjectionCoordinates &coord) const
{
    computeCoordinatesHelper(cellSizes_, projection, coord);
}

void ompl::base::ProjectionEvaluator::printSettings(std::ostream &out) const
{
    out << "Projection of dimension " << getDimension() << std::endl;
    out << "Cell sizes";
    if (cellSizesWereInferred_)
        out << " (inferred by sampling)";
    else
    {
        if (defaultCellSizes_)
            out << " (computed defaults)";
        else
            out << " (set by user)";
    }
    out << ": [";
    for (unsigned int i = 0 ; i < cellSizes_.size() ; ++i)
    {
        out << cellSizes_[i];
        if (i + 1 < cellSizes_.size())
            out << ' ';
    }
    out << ']' << std::endl;
}

void ompl::base::ProjectionEvaluator::printProjection(const EuclideanProjection &projection, std::ostream &out) const
{
    unsigned int d = getDimension();
    if (d > 0)
    {
        --d;
        for (unsigned int i = 0 ; i < d ; ++i)
            out << projection.values[i] << ' ';
        out << projection.values[d] << std::endl;
    }
    else
        out << "NULL" << std::endl;
}
