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

#include "ompl/base/manifolds/RealVectorStateProjections.h"
#include "ompl/util/Exception.h"
#include "ompl/util/MagicConstants.h"
#include <cstring>

/// @cond IGNORE
namespace ompl
{
    namespace base
    {
        static inline void checkManifoldType(const StateManifold *m)
        {
            if (!dynamic_cast<const RealVectorStateManifold*>(m))
                throw Exception("Expected real vector manifold for projection");
        }
    }
}
/// @endcond

ompl::base::RealVectorLinearProjectionEvaluator::RealVectorLinearProjectionEvaluator(const StateManifold *manifold, const std::vector<double> &cellSizes,
                                                                                     const ProjectionMatrix::Matrix &projection) :
    ProjectionEvaluator(manifold)
{
    checkManifoldType(manifold_);
    projection_.mat = projection;
    setCellSizes(cellSizes);
}

ompl::base::RealVectorLinearProjectionEvaluator::RealVectorLinearProjectionEvaluator(const StateManifoldPtr &manifold, const std::vector<double> &cellSizes,
                                                                                     const ProjectionMatrix::Matrix &projection) :
    ProjectionEvaluator(manifold)
{
    checkManifoldType(manifold_);
    projection_.mat = projection;
    setCellSizes(cellSizes);
}

ompl::base::RealVectorLinearProjectionEvaluator::RealVectorLinearProjectionEvaluator(const StateManifold *manifold,
                                                                                     const ProjectionMatrix::Matrix &projection) :
    ProjectionEvaluator(manifold)
{
    checkManifoldType(manifold_);
    projection_.mat = projection;
}

ompl::base::RealVectorLinearProjectionEvaluator::RealVectorLinearProjectionEvaluator(const StateManifoldPtr &manifold,
                                                                                     const ProjectionMatrix::Matrix &projection) :
    ProjectionEvaluator(manifold)
{
    checkManifoldType(manifold_);
    projection_.mat = projection;
}

ompl::base::RealVectorOrthogonalProjectionEvaluator::RealVectorOrthogonalProjectionEvaluator(const StateManifold *manifold, const std::vector<double> &cellSizes,
                                                                                             const std::vector<unsigned int> &components) :
    ProjectionEvaluator(manifold), components_(components)
{
    checkManifoldType(manifold_);
    setCellSizes(cellSizes);
}

ompl::base::RealVectorOrthogonalProjectionEvaluator::RealVectorOrthogonalProjectionEvaluator(const StateManifoldPtr &manifold, const std::vector<double> &cellSizes,
                                                                                             const std::vector<unsigned int> &components) :
    ProjectionEvaluator(manifold), components_(components)
{
    checkManifoldType(manifold_);
    setCellSizes(cellSizes);
}

ompl::base::RealVectorOrthogonalProjectionEvaluator::RealVectorOrthogonalProjectionEvaluator(const StateManifold *manifold, const std::vector<unsigned int> &components) :
    ProjectionEvaluator(manifold), components_(components)
{
    checkManifoldType(manifold_);
}

ompl::base::RealVectorOrthogonalProjectionEvaluator::RealVectorOrthogonalProjectionEvaluator(const StateManifoldPtr &manifold, const std::vector<unsigned int> &components) :
    ProjectionEvaluator(manifold), components_(components)
{
    checkManifoldType(manifold_);
}

void ompl::base::RealVectorOrthogonalProjectionEvaluator::defaultCellSizes(void)
{
    const RealVectorBounds &bounds = manifold_->as<RealVectorStateManifold>()->getBounds();
    cellSizes_.resize(components_.size());
    for (unsigned int i = 0 ; i < cellSizes_.size() ; ++i)
        cellSizes_[i] = (bounds.high[components_[i]] - bounds.low[components_[i]]) / magic::PROJECTION_DIMENSION_SPLITS;
}

unsigned int ompl::base::RealVectorLinearProjectionEvaluator::getDimension(void) const
{
    return projection_.mat.size();
}

void ompl::base::RealVectorLinearProjectionEvaluator::project(const State *state, EuclideanProjection &projection) const
{
    projection_.project(state->as<RealVectorStateManifold::StateType>()->values, projection.values);
}

unsigned int ompl::base::RealVectorOrthogonalProjectionEvaluator::getDimension(void) const
{
    return components_.size();
}

void ompl::base::RealVectorOrthogonalProjectionEvaluator::project(const State *state, EuclideanProjection &projection) const
{
    for (unsigned int i = 0 ; i < components_.size() ; ++i)
        projection.values[i] = state->as<RealVectorStateManifold::StateType>()->values[components_[i]];
}

ompl::base::RealVectorIdentityProjectionEvaluator::RealVectorIdentityProjectionEvaluator(const StateManifold *manifold, const std::vector<double> &cellSizes) :
    ProjectionEvaluator(manifold)
{
    checkManifoldType(manifold_);
    setCellSizes(cellSizes);
}

ompl::base::RealVectorIdentityProjectionEvaluator::RealVectorIdentityProjectionEvaluator(const StateManifold *manifold) :
    ProjectionEvaluator(manifold)
{
    checkManifoldType(manifold_);
}

ompl::base::RealVectorIdentityProjectionEvaluator::RealVectorIdentityProjectionEvaluator(const StateManifoldPtr &manifold, const std::vector<double> &cellSizes) :
    ProjectionEvaluator(manifold)
{
    checkManifoldType(manifold_);
    setCellSizes(cellSizes);
}

ompl::base::RealVectorIdentityProjectionEvaluator::RealVectorIdentityProjectionEvaluator(const StateManifoldPtr &manifold) :
    ProjectionEvaluator(manifold)
{
    checkManifoldType(manifold_);
}

void ompl::base::RealVectorIdentityProjectionEvaluator::defaultCellSizes(void)
{
    const RealVectorBounds &bounds = manifold_->as<RealVectorStateManifold>()->getBounds();
    cellSizes_.resize(getDimension());
    for (unsigned int i = 0 ; i < cellSizes_.size() ; ++i)
        cellSizes_[i] = (bounds.high[i] - bounds.low[i]) / magic::PROJECTION_DIMENSION_SPLITS;
}

void ompl::base::RealVectorIdentityProjectionEvaluator::setup(void)
{
    copySize_ = getDimension() * sizeof(double);
    ProjectionEvaluator::setup();
}

unsigned int ompl::base::RealVectorIdentityProjectionEvaluator::getDimension(void) const
{
    return manifold_->getDimension();
}

void ompl::base::RealVectorIdentityProjectionEvaluator::project(const State *state, EuclideanProjection &projection) const
{
    memcpy(projection.values, state->as<RealVectorStateManifold::StateType>()->values, copySize_);
}
