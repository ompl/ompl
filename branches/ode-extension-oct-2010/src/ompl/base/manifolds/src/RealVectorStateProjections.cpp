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
#include "ompl/util/RandomNumbers.h"
#include <cstring>

ompl::base::RealVectorLinearProjectionEvaluator::RealVectorLinearProjectionEvaluator(const StateManifold *manifold,
										     const std::vector<double> &cellDimensions,
										     const std::vector< std::valarray<double> > &projection) :
    ProjectionEvaluator(manifold), projection_(projection)
{
    if (!dynamic_cast<const RealVectorStateManifold*>(manifold_))
	throw Exception("Expected real vector manifold for projection");
    setCellDimensions(cellDimensions);
}

ompl::base::RealVectorLinearProjectionEvaluator::RealVectorLinearProjectionEvaluator(const StateManifoldPtr &manifold,
										     const std::vector<double> &cellDimensions,
										     const std::vector< std::valarray<double> > &projection) :
    ProjectionEvaluator(manifold.get()), projection_(projection)
{
    if (!dynamic_cast<const RealVectorStateManifold*>(manifold_))
	throw Exception("Expected real vector manifold for projection");
    setCellDimensions(cellDimensions);
}

ompl::base::RealVectorOrthogonalProjectionEvaluator::RealVectorOrthogonalProjectionEvaluator(const StateManifold *manifold,
											     const std::vector<double> &cellDimensions,
											     const std::vector<unsigned int> &components) : 
    ProjectionEvaluator(manifold), components_(components)
{
    if (!dynamic_cast<const RealVectorStateManifold*>(manifold_))
	throw Exception("Expected real vector manifold for projection");
    setCellDimensions(cellDimensions);
}

ompl::base::RealVectorOrthogonalProjectionEvaluator::RealVectorOrthogonalProjectionEvaluator(const StateManifoldPtr &manifold,
											     const std::vector<double> &cellDimensions,
											     const std::vector<unsigned int> &components) : 
    ProjectionEvaluator(manifold.get()), components_(components)
{
    if (!dynamic_cast<const RealVectorStateManifold*>(manifold_))
	throw Exception("Expected real vector manifold for projection");
    setCellDimensions(cellDimensions);
}

void ompl::base::RealVectorLinearProjectionEvaluator::project(const State *state, EuclideanProjection &projection) const
{
    for (unsigned int i = 0 ; i < projection_.size() ; ++i)
    {
	const std::valarray<double> &vec = projection_[i];
	const unsigned int dim = vec.size();
	double *pos = projection.values + i;
	const double *values = state->as<RealVectorStateManifold::StateType>()->values;
	*pos = 0.0;
	for (unsigned int j = 0 ; j < dim ; ++j)
	    *pos += values[j] * vec[j];
    }
}

void ompl::base::RealVectorOrthogonalProjectionEvaluator::project(const State *state, EuclideanProjection &projection) const
{
    for (unsigned int i = 0 ; i < components_.size() ; ++i)
	projection.values[i] = state->as<RealVectorStateManifold::StateType>()->values[components_[i]];
}

std::vector< std::valarray<double> > ompl::base::RealVectorRandomLinearProjectionEvaluator::computeProjection(unsigned int from, unsigned int to) const
{
    RNG rng;
    
    std::vector< std::valarray<double> > p(to);
    for (unsigned int i = 0 ; i < p.size() ; ++i)
    {
	p[i].resize(from);
	for (unsigned int j = 0 ; j < p[i].size() ; ++j)
	    p[i][j] = rng.gaussian01();	
    }

    for (unsigned int i = 1 ; i < p.size() ; ++i)
    {
	std::valarray<double> &row = p[i];
	for (unsigned int j = 0 ; j < i ; ++j)
	{
	    std::valarray<double> &prevRow = p[j];
	    // subtract projection
	    row -= (row * prevRow).sum() * prevRow;
	}
	// normalize
	row /= sqrt((row * row).sum());	
    }
        
    return p;
}

ompl::base::RealVectorIdentityProjectionEvaluator::RealVectorIdentityProjectionEvaluator(const StateManifold *manifold,
											 const std::vector<double> &cellDimensions) :
    ProjectionEvaluator(manifold)
{
    if (!dynamic_cast<const RealVectorStateManifold*>(manifold_))
	throw Exception("Expected real vector manifold for projection");
    setCellDimensions(cellDimensions);
    copySize_ = manifold_->getDimension() * sizeof(double);
}

ompl::base::RealVectorIdentityProjectionEvaluator::RealVectorIdentityProjectionEvaluator(const StateManifoldPtr &manifold,
											 const std::vector<double> &cellDimensions) :
    ProjectionEvaluator(manifold.get())
{
    if (!dynamic_cast<const RealVectorStateManifold*>(manifold_))
	throw Exception("Expected real vector manifold for projection");
    setCellDimensions(cellDimensions);
    copySize_ = manifold_->getDimension() * sizeof(double);
}

void ompl::base::RealVectorIdentityProjectionEvaluator::project(const State *state, EuclideanProjection &projection) const
{
    memcpy(projection.values, state->as<RealVectorStateManifold::StateType>()->values, copySize_);
}
