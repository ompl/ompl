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
#include <functional>
#include <cmath>
#include <cstring>
#include <limits>

ompl::base::ProjectionMatrix::Matrix ompl::base::ProjectionMatrix::ComputeRandom(const unsigned int from,
                                                                                 const unsigned int to,
                                                                                 const std::vector<double> &scale)
{
    namespace nu = boost::numeric::ublas;

    RNG rng;
    Matrix projection(to, from);

    for (unsigned int j = 0; j < from; ++j)
    {
        if (scale.size() == from && fabs(scale[j]) < std::numeric_limits<double>::epsilon())
            nu::column(projection, j) = nu::zero_vector<double>(to);
        else
            for (unsigned int i = 0; i < to; ++i)
                projection(i, j) = rng.gaussian01();
    }

    for (unsigned int i = 0; i < to; ++i)
    {
        nu::matrix_row<Matrix> row(projection, i);
        for (unsigned int j = 0; j < i; ++j)
        {
            nu::matrix_row<Matrix> prevRow(projection, j);
            // subtract projection
            row -= inner_prod(row, prevRow) * prevRow;
        }
        // normalize
        row /= norm_2(row);
    }

    assert(scale.size() == from || scale.size() == 0);
    if (scale.size() == from)
    {
        unsigned int z = 0;
        for (unsigned int i = 0; i < from; ++i)
        {
            if (fabs(scale[i]) < std::numeric_limits<double>::epsilon())
                z++;
            else
                nu::column(projection, i) /= scale[i];
        }
        if (z == from)
            OMPL_WARN("Computed projection matrix is all 0s");
    }
    return projection;
}

ompl::base::ProjectionMatrix::Matrix ompl::base::ProjectionMatrix::ComputeRandom(const unsigned int from,
                                                                                 const unsigned int to)
{
    return ComputeRandom(from, to, std::vector<double>());
}

void ompl::base::ProjectionMatrix::computeRandom(const unsigned int from, const unsigned int to,
                                                 const std::vector<double> &scale)
{
    mat = ComputeRandom(from, to, scale);
}

void ompl::base::ProjectionMatrix::computeRandom(const unsigned int from, const unsigned int to)
{
    mat = ComputeRandom(from, to);
}

void ompl::base::ProjectionMatrix::project(const double *from, EuclideanProjection &to) const
{
    namespace nu = boost::numeric::ublas;
    // create a temporary uBLAS vector from a C-style array without copying data
    nu::shallow_array_adaptor<const double> tmp1(mat.size2(), from);
    nu::vector<double, nu::shallow_array_adaptor<const double>> tmp2(mat.size2(), tmp1);
    to = prod(mat, tmp2);
}

void ompl::base::ProjectionMatrix::print(std::ostream &out) const
{
    out << mat << std::endl;
}

ompl::base::ProjectionEvaluator::ProjectionEvaluator(const StateSpace *space)
  : space_(space), bounds_(0), estimatedBounds_(0), defaultCellSizes_(true), cellSizesWereInferred_(false)
{
    params_.declareParam<double>("cellsize_factor",
                                 std::bind(&ProjectionEvaluator::mulCellSizes, this, std::placeholders::_1));
}

ompl::base::ProjectionEvaluator::ProjectionEvaluator(const StateSpacePtr &space)
  : space_(space.get()), bounds_(0), estimatedBounds_(0), defaultCellSizes_(true), cellSizesWereInferred_(false)
{
    params_.declareParam<double>("cellsize_factor",
                                 std::bind(&ProjectionEvaluator::mulCellSizes, this, std::placeholders::_1));
}

ompl::base::ProjectionEvaluator::~ProjectionEvaluator()
{
}

bool ompl::base::ProjectionEvaluator::userConfigured() const
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

void ompl::base::ProjectionEvaluator::setBounds(const RealVectorBounds &bounds)
{
    bounds_ = bounds;
    checkBounds();
}

void ompl::base::ProjectionEvaluator::setCellSizes(unsigned int dim, double cellSize)
{
    if (cellSizes_.size() >= dim)
        OMPL_ERROR("Dimension %u is not defined for projection evaluator", dim);
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
    OMPL_ERROR("Dimension %u is not defined for projection evaluator", dim);
    return 0.0;
}

void ompl::base::ProjectionEvaluator::mulCellSizes(double factor)
{
    if (cellSizes_.size() == getDimension())
    {
        std::vector<double> c(cellSizes_.size());
        for (std::size_t i = 0; i < cellSizes_.size(); ++i)
            c[i] = cellSizes_[i] * factor;
        setCellSizes(c);
    }
}

void ompl::base::ProjectionEvaluator::checkCellSizes() const
{
    if (getDimension() <= 0)
        throw Exception("Dimension of projection needs to be larger than 0");
    if (cellSizes_.size() != getDimension())
        throw Exception("Number of dimensions in projection space does not match number of cell sizes");
}

void ompl::base::ProjectionEvaluator::checkBounds() const
{
    bounds_.check();
    if (hasBounds() && bounds_.low.size() != getDimension())
        throw Exception("Number of dimensions in projection space does not match dimension of bounds");
}

void ompl::base::ProjectionEvaluator::defaultCellSizes()
{
}

/// @cond IGNORE
namespace ompl
{
    namespace base
    {
        static inline void computeCoordinatesHelper(const std::vector<double> &cellSizes,
                                                    const EuclideanProjection &projection, ProjectionCoordinates &coord)
        {
            const std::size_t dim = cellSizes.size();
            coord.resize(dim);
            for (unsigned int i = 0; i < dim; ++i)
                coord[i] = (int)floor(projection(i) / cellSizes[i]);
        }
    }
}
/// @endcond

void ompl::base::ProjectionEvaluator::inferBounds()
{
    if (estimatedBounds_.low.empty())
        estimateBounds();
    bounds_ = estimatedBounds_;
}

void ompl::base::ProjectionEvaluator::estimateBounds()
{
    unsigned int dim = getDimension();
    estimatedBounds_.resize(dim);
    if (dim > 0)
    {
        StateSamplerPtr sampler = space_->allocStateSampler();
        State *s = space_->allocState();
        EuclideanProjection proj(dim);

        estimatedBounds_.setLow(std::numeric_limits<double>::infinity());
        estimatedBounds_.setHigh(-std::numeric_limits<double>::infinity());

        for (unsigned int i = 0; i < magic::PROJECTION_EXTENTS_SAMPLES; ++i)
        {
            sampler->sampleUniform(s);
            project(s, proj);
            for (unsigned int j = 0; j < dim; ++j)
            {
                if (estimatedBounds_.low[j] > proj[j])
                    estimatedBounds_.low[j] = proj[j];
                if (estimatedBounds_.high[j] < proj[j])
                    estimatedBounds_.high[j] = proj[j];
            }
        }
        // make bounding box 10% larger (5% padding on each side)
        std::vector<double> diff(estimatedBounds_.getDifference());
        for (unsigned int j = 0; j < dim; ++j)
        {
            estimatedBounds_.low[j] -= magic::PROJECTION_EXPAND_FACTOR * diff[j];
            estimatedBounds_.high[j] += magic::PROJECTION_EXPAND_FACTOR * diff[j];
        }

        space_->freeState(s);
    }
}

void ompl::base::ProjectionEvaluator::inferCellSizes()
{
    cellSizesWereInferred_ = true;
    if (!hasBounds())
        inferBounds();
    unsigned int dim = getDimension();
    cellSizes_.resize(dim);
    for (unsigned int j = 0; j < dim; ++j)
    {
        cellSizes_[j] = (bounds_.high[j] - bounds_.low[j]) / magic::PROJECTION_DIMENSION_SPLITS;
        if (cellSizes_[j] < std::numeric_limits<double>::epsilon())
        {
            cellSizes_[j] = 1.0;
            OMPL_WARN("Inferred cell size for dimension %u of a projection for state space %s is 0. Setting arbitrary "
                      "value of 1 instead.",
                      j, space_->getName().c_str());
        }
    }
}

void ompl::base::ProjectionEvaluator::setup()
{
    typedef void (ProjectionEvaluator::*setCellSizesFunctionType)(unsigned int, double);
    typedef double (ProjectionEvaluator::*getCellSizesFunctionType)(unsigned int) const;

    if (defaultCellSizes_)
        defaultCellSizes();

    if ((cellSizes_.size() == 0 && getDimension() > 0) || cellSizesWereInferred_)
        inferCellSizes();

    checkCellSizes();
    checkBounds();

    unsigned int dim = getDimension();
    for (unsigned int i = 0; i < dim; ++i)
        params_.declareParam<double>(
            "cellsize." + std::to_string(i),
            std::bind((setCellSizesFunctionType)&ProjectionEvaluator::setCellSizes, this, i, std::placeholders::_1),
            std::bind((getCellSizesFunctionType)&ProjectionEvaluator::getCellSizes, this, i));
}

void ompl::base::ProjectionEvaluator::computeCoordinates(const EuclideanProjection &projection,
                                                         ProjectionCoordinates &coord) const
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
    for (unsigned int i = 0; i < cellSizes_.size(); ++i)
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

ompl::base::SubspaceProjectionEvaluator::SubspaceProjectionEvaluator(const StateSpace *space, unsigned int index,
                                                                     const ProjectionEvaluatorPtr &projToUse)
  : ProjectionEvaluator(space), index_(index), specifiedProj_(projToUse)
{
    if (!space_->isCompound())
        throw Exception("Cannot construct a subspace projection evaluator for a space that is not compound");
    if (space_->as<CompoundStateSpace>()->getSubspaceCount() <= index_)
        throw Exception("State space " + space_->getName() + " does not have a subspace at index " +
                        std::to_string(index_));
}

void ompl::base::SubspaceProjectionEvaluator::setup()
{
    if (specifiedProj_)
        proj_ = specifiedProj_;
    else
        proj_ = space_->as<CompoundStateSpace>()->getSubspace(index_)->getDefaultProjection();
    if (!proj_)
        throw Exception("No projection specified for subspace at index " + std::to_string(index_));

    cellSizes_ = proj_->getCellSizes();
    ProjectionEvaluator::setup();
}

unsigned int ompl::base::SubspaceProjectionEvaluator::getDimension() const
{
    return proj_->getDimension();
}

void ompl::base::SubspaceProjectionEvaluator::project(const State *state, EuclideanProjection &projection) const
{
    proj_->project(state->as<CompoundState>()->components[index_], projection);
}
