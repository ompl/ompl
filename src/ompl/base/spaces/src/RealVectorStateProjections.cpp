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

#include "ompl/base/spaces/RealVectorStateProjections.h"
#include "ompl/util/Exception.h"
#include "ompl/tools/config/MagicConstants.h"
#include <cstring>
#include <utility>

/// @cond IGNORE
namespace ompl
{
    namespace base
    {
        static inline void checkSpaceType(const StateSpace *m)
        {
            if (dynamic_cast<const RealVectorStateSpace *>(m) == nullptr)
                throw Exception("Expected real vector state space for projection");
        }
    }  // namespace base
}  // namespace ompl
/// @endcond

ompl::base::RealVectorLinearProjectionEvaluator::RealVectorLinearProjectionEvaluator(
    const StateSpace *space, const std::vector<double> &cellSizes, const ProjectionMatrix::Matrix &projection)
  : ProjectionEvaluator(space)
{
    checkSpaceType(space_);
    projection_.mat = projection;
    setCellSizes(cellSizes);
}

ompl::base::RealVectorLinearProjectionEvaluator::RealVectorLinearProjectionEvaluator(
    const StateSpacePtr &space, const std::vector<double> &cellSizes, const ProjectionMatrix::Matrix &projection)
  : ProjectionEvaluator(space)
{
    checkSpaceType(space_);
    projection_.mat = projection;
    setCellSizes(cellSizes);
}

ompl::base::RealVectorLinearProjectionEvaluator::RealVectorLinearProjectionEvaluator(
    const StateSpace *space, const ProjectionMatrix::Matrix &projection)
  : ProjectionEvaluator(space)
{
    checkSpaceType(space_);
    projection_.mat = projection;
}

ompl::base::RealVectorLinearProjectionEvaluator::RealVectorLinearProjectionEvaluator(
    const StateSpacePtr &space, const ProjectionMatrix::Matrix &projection)
  : ProjectionEvaluator(space)
{
    checkSpaceType(space_);
    projection_.mat = projection;
}

ompl::base::RealVectorOrthogonalProjectionEvaluator::RealVectorOrthogonalProjectionEvaluator(
    const StateSpace *space, const std::vector<double> &cellSizes, std::vector<unsigned int> components)
  : ProjectionEvaluator(space), components_(std::move(components))
{
    checkSpaceType(space_);
    setCellSizes(cellSizes);
    copyBounds();
}

ompl::base::RealVectorOrthogonalProjectionEvaluator::RealVectorOrthogonalProjectionEvaluator(
    const StateSpacePtr &space, const std::vector<double> &cellSizes, std::vector<unsigned int> components)
  : ProjectionEvaluator(space), components_(std::move(components))
{
    checkSpaceType(space_);
    setCellSizes(cellSizes);
    copyBounds();
}

ompl::base::RealVectorOrthogonalProjectionEvaluator::RealVectorOrthogonalProjectionEvaluator(
    const StateSpace *space, std::vector<unsigned int> components)
  : ProjectionEvaluator(space), components_(std::move(components))
{
    checkSpaceType(space_);
}

ompl::base::RealVectorOrthogonalProjectionEvaluator::RealVectorOrthogonalProjectionEvaluator(
    const StateSpacePtr &space, std::vector<unsigned int> components)
  : ProjectionEvaluator(space), components_(std::move(components))
{
    checkSpaceType(space_);
}

void ompl::base::RealVectorOrthogonalProjectionEvaluator::copyBounds()
{
    bounds_.resize(components_.size());
    const RealVectorBounds &bounds = space_->as<RealVectorStateSpace>()->getBounds();
    for (unsigned int i = 0; i < components_.size(); ++i)
    {
        bounds_.low[i] = bounds.low[components_[i]];
        bounds_.high[i] = bounds.high[components_[i]];
    }
}

void ompl::base::RealVectorOrthogonalProjectionEvaluator::defaultCellSizes()
{
    const RealVectorBounds &bounds = space_->as<RealVectorStateSpace>()->getBounds();
    bounds_.resize(components_.size());
    cellSizes_.resize(components_.size());
    for (unsigned int i = 0; i < cellSizes_.size(); ++i)
    {
        bounds_.low[i] = bounds.low[components_[i]];
        bounds_.high[i] = bounds.high[components_[i]];
        cellSizes_[i] = (bounds_.high[i] - bounds_.low[i]) / magic::PROJECTION_DIMENSION_SPLITS;
    }
}

unsigned int ompl::base::RealVectorLinearProjectionEvaluator::getDimension() const
{
    return projection_.mat.rows();
}

void ompl::base::RealVectorLinearProjectionEvaluator::project(const State *state,
                                                              Eigen::Ref<Eigen::VectorXd> projection) const
{
    projection_.project(state->as<RealVectorStateSpace::StateType>()->values, projection);
}

unsigned int ompl::base::RealVectorOrthogonalProjectionEvaluator::getDimension() const
{
    return components_.size();
}

void ompl::base::RealVectorOrthogonalProjectionEvaluator::project(const State *state,
                                                                  Eigen::Ref<Eigen::VectorXd> projection) const
{
    for (unsigned int i = 0; i < components_.size(); ++i)
        projection(i) = state->as<RealVectorStateSpace::StateType>()->values[components_[i]];
}

ompl::base::RealVectorIdentityProjectionEvaluator::RealVectorIdentityProjectionEvaluator(
    const StateSpace *space, const std::vector<double> &cellSizes)
  : ProjectionEvaluator(space)
{
    checkSpaceType(space_);
    setCellSizes(cellSizes);
    copyBounds();
}

ompl::base::RealVectorIdentityProjectionEvaluator::RealVectorIdentityProjectionEvaluator(const StateSpace *space)
  : ProjectionEvaluator(space)
{
    checkSpaceType(space_);
}

ompl::base::RealVectorIdentityProjectionEvaluator::RealVectorIdentityProjectionEvaluator(
    const StateSpacePtr &space, const std::vector<double> &cellSizes)
  : ProjectionEvaluator(space)
{
    checkSpaceType(space_);
    setCellSizes(cellSizes);
    copyBounds();
}

ompl::base::RealVectorIdentityProjectionEvaluator::RealVectorIdentityProjectionEvaluator(const StateSpacePtr &space)
  : ProjectionEvaluator(space)
{
    checkSpaceType(space_);
}

void ompl::base::RealVectorIdentityProjectionEvaluator::copyBounds()
{
    bounds_ = space_->as<RealVectorStateSpace>()->getBounds();
}

void ompl::base::RealVectorIdentityProjectionEvaluator::defaultCellSizes()
{
    bounds_ = space_->as<RealVectorStateSpace>()->getBounds();
    cellSizes_.resize(getDimension());
    for (unsigned int i = 0; i < cellSizes_.size(); ++i)
        cellSizes_[i] = (bounds_.high[i] - bounds_.low[i]) / magic::PROJECTION_DIMENSION_SPLITS;
}

void ompl::base::RealVectorIdentityProjectionEvaluator::setup()
{
    copySize_ = getDimension();
    ProjectionEvaluator::setup();
}

unsigned int ompl::base::RealVectorIdentityProjectionEvaluator::getDimension() const
{
    return space_->getDimension();
}

void ompl::base::RealVectorIdentityProjectionEvaluator::project(const State *state,
                                                                Eigen::Ref<Eigen::VectorXd> projection) const
{
    projection = Eigen::Map<const Eigen::VectorXd>(state->as<RealVectorStateSpace::StateType>()->values, copySize_);
}
