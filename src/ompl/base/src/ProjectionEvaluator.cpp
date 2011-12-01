/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage, Inc.
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

// We need this to create a temporary uBLAS vector from a C-style array without copying data
#define BOOST_UBLAS_SHALLOW_ARRAY_ADAPTOR
#include "ompl/base/StateSpace.h"
#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/util/Exception.h"
#include "ompl/util/RandomNumbers.h"
#include "ompl/tools/config/MagicConstants.h"
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/bind.hpp>
#include <cmath>
#include <cstring>
#include <limits>

// static const double DIMENSION_UPDATE_FACTOR = 1.2;

ompl::base::ProjectionMatrix::Matrix ompl::base::ProjectionMatrix::ComputeRandom(const unsigned int from, const unsigned int to, const std::vector<double> &scale)
{
    using namespace boost::numeric::ublas;

    RNG rng;
    Matrix projection(to, from);

    for (unsigned int i = 0 ; i < to ; ++i)
    {
        for (unsigned int j = 0 ; j < from ; ++j)
            projection(i, j) = rng.gaussian01();
    }

    for (unsigned int i = 1 ; i < to ; ++i)
    {
        matrix_row<Matrix> row(projection, i);
        for (unsigned int j = 0 ; j < i ; ++j)
        {
            matrix_row<Matrix> prevRow(projection, j);
            // subtract projection
            row -= inner_prod(row, prevRow) * prevRow;
        }
        // normalize
        row /= norm_2(row);
    }

    assert(scale.size() == from || scale.size() == 0);
    if (scale.size() == from)
        for (unsigned int i = 0 ; i < from ; ++i)
        {
            if (fabs(scale[i]) < std::numeric_limits<double>::epsilon())
                throw Exception("Scaling factor must be non-zero");
            boost::numeric::ublas::column(projection, i) /= scale[i];
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

void ompl::base::ProjectionMatrix::project(const double *from, EuclideanProjection& to) const
{
    using namespace boost::numeric::ublas;
    // create a temporary uBLAS vector from a C-style array without copying data
    shallow_array_adaptor<const double> tmp1(mat.size2(), from);
    vector<double, shallow_array_adaptor<const double> > tmp2(mat.size2(), tmp1);
    to = prod(mat, tmp2);
}

void ompl::base::ProjectionMatrix::print(std::ostream &out) const
{
    out << mat << std::endl;
}

ompl::base::ProjectionEvaluator::ProjectionEvaluator(const StateSpace *space) : space_(space), defaultCellSizes_(true), cellSizesWereInferred_(false)
{
    params_.declareParam<double>("cellsize_factor", boost::bind(&ProjectionEvaluator::mulCellSizes, this, _1));
}

ompl::base::ProjectionEvaluator::ProjectionEvaluator(const StateSpacePtr &space) : space_(space.get()), defaultCellSizes_(true), cellSizesWereInferred_(false)
{
    params_.declareParam<double>("cellsize_factor", boost::bind(&ProjectionEvaluator::mulCellSizes, this, _1));
}

ompl::base::ProjectionEvaluator::~ProjectionEvaluator(void)
{
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

void ompl::base::ProjectionEvaluator::setCellSizes(unsigned int dim, double cellSize)
{
    if (cellSizes_.size() >= dim)
        msg_.error("Dimension %u is not defined for projection evaluator", dim);
    else
    {
        std::vector<double> c = cellSizes_;
        c[dim] = cellSize;
        setCellSizes(c);
    }
}

double ompl::base::ProjectionEvaluator::getCellSizes(unsigned int dim) const
{
    if (cellSizes_.size() > dim)
        return cellSizes_[dim];
    msg_.error("Dimension %u is not defined for projection evaluator", dim);
    return 0.0;
}

void ompl::base::ProjectionEvaluator::mulCellSizes(double factor)
{
    if (cellSizes_.size() == getDimension())
    {
        std::vector<double> c(cellSizes_.size());
        for (std::size_t i = 0 ; i < cellSizes_.size() ; ++i)
            c[i] = cellSizes_[i] * factor;
        setCellSizes(c);
    }
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
                coord[i] = (int)floor(projection(i)/cellSizes[i]);
        }
        /*
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
        */
    }
}
/// @endcond



/*
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
*/

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
                if (low[j] > proj[j])
                    low[j] = proj[j];
                if (high[j] < proj[j])
                    high[j] = proj[j];
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

    unsigned int dim = getDimension();
    for (unsigned int i = 0 ; i < dim ; ++i)
        params_.declareParam<double>("cellsize." + boost::lexical_cast<std::string>(i),
                                     boost::bind(&ProjectionEvaluator::setCellSizes, this, i, _1),
                                     boost::bind(&ProjectionEvaluator::getCellSizes, this, i));
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
    out << projection << std::endl;
}

ompl::base::SubSpaceProjectionEvaluator::SubSpaceProjectionEvaluator(const StateSpace *space, unsigned int index, const ProjectionEvaluatorPtr &projToUse) :
    ProjectionEvaluator(space), index_(index), specifiedProj_(projToUse)
{
    if (!space_->isCompound())
        throw Exception("Cannot construct a subspace projection evaluator for a space that is not compound");
    if (space_->as<CompoundStateSpace>()->getSubSpaceCount() >= index_)
        throw Exception("State space " + space_->getName() + " does not have a subspace at index " + boost::lexical_cast<std::string>(index_));
}

void ompl::base::SubSpaceProjectionEvaluator::setup(void)
{
    if (specifiedProj_)
        proj_ = specifiedProj_;
    else
        proj_ = space_->as<CompoundStateSpace>()->getSubSpace(index_)->getDefaultProjection();
    if (!proj_)
        throw Exception("No projection specified for subspace at index " + boost::lexical_cast<std::string>(index_));

    cellSizes_ = proj_->getCellSizes();
    ProjectionEvaluator::setup();
}

unsigned int ompl::base::SubSpaceProjectionEvaluator::getDimension(void) const
{
    return proj_->getDimension();
}

void ompl::base::SubSpaceProjectionEvaluator::project(const State *state, EuclideanProjection &projection) const
{
    proj_->project(state->as<CompoundState>()->components[index_], projection);
}
