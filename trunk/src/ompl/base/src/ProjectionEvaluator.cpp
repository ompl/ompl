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

#include "ompl/base/StateManifold.h"
#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/util/Exception.h"
#include "ompl/util/RandomNumbers.h"
#include <cmath>
#include <cstring>

ompl::base::ProjectionMatrix::Matrix ompl::base::ProjectionMatrix::ComputeRandom(const unsigned int from, const unsigned int to)
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
    
    return projection;
}

void ompl::base::ProjectionMatrix::computeRandom(const unsigned int from, const unsigned int to)
{
    projection = ComputeRandom(from, to);
}

void ompl::base::ProjectionMatrix::project(const double *from, double *to) const
{
    for (unsigned int i = 0 ; i < projection.size() ; ++i)
    {
	const std::valarray<double> &vec = projection[i];
	const unsigned int dim = vec.size();
	double *pos = to + i;
	*pos = 0.0;
	for (unsigned int j = 0 ; j < dim ; ++j)
	    *pos += from[j] * vec[j];
    }
}

void ompl::base::ProjectionEvaluator::setCellDimensions(const std::vector<double> &cellDimensions)
{
    cellDimensions_ = cellDimensions;
    checkCellDimensions();
}

void ompl::base::ProjectionEvaluator::checkCellDimensions(void) const
{
    if (getDimension() <= 0)
	throw Exception("Dimension of projection needs to be larger than 0");
    if (cellDimensions_.size() != getDimension())
	throw Exception("Number of dimensions in projection space does not match number of cell dimensions");
}

void ompl::base::ProjectionEvaluator::computeCoordinates(const EuclideanProjection &projection, ProjectionCoordinates &coord) const
{
    unsigned int dim = getDimension();
    coord.resize(dim);
    for (unsigned int i = 0 ; i < dim ; ++i)
	coord[i] = (int)floor(projection.values[i]/cellDimensions_[i]);
}

void ompl::base::ProjectionEvaluator::printSettings(std::ostream &out) const
{ 
    out << "Projection of dimension " << getDimension() << std::endl;
    out << "Cell dimensions: [";
    for (unsigned int i = 0 ; i < cellDimensions_.size() ; ++i)
    {
	out << cellDimensions_[i];
	if (i + 1 < cellDimensions_.size())
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

void ompl::base::CompoundProjectionEvaluator::addProjectionEvaluator(const ProjectionEvaluatorPtr &proj)
{
    components_.push_back(proj);
    computeProjection();
}

void ompl::base::CompoundProjectionEvaluator::computeProjection(void)
{
    compoundDimension_ = 0;
    std::vector<double> cdims;
    for (unsigned int i = 0 ; i < components_.size() ; ++i)
    {
	std::vector<double> d = components_[i]->getCellDimensions();
	cdims.insert(cdims.end(), d.begin(), d.end());
	compoundDimension_ += components_[i]->getDimension();
    }
    
    if (compoundDimension_ > 2 && components_.size() > 1)
	dimension_ = std::max(2, (int)ceil(log((double)compoundDimension_)));
    else
	dimension_ = compoundDimension_;
    if (dimension_ < compoundDimension_)
    {
	projection_.computeRandom(compoundDimension_, dimension_);
	cellDimensions_.resize(dimension_);
	projection_.project(&cdims[0], &cellDimensions_[0]);
    }
    else
	cellDimensions_ = cdims;
}

unsigned int ompl::base::CompoundProjectionEvaluator::getDimension(void) const
{
    return dimension_;
}

void ompl::base::CompoundProjectionEvaluator::project(const State *state, EuclideanProjection &projection) const
{
    // project all states to one big vector
    EuclideanProjection all(compoundDimension_);
    double *start = all.values;
    unsigned int p = 0;
    for (unsigned int i = 0 ; i < components_.size() ; ++i)
    {
	components_[i]->project(state->as<CompoundState>()->components[i], all);
	all.values += components_[i]->getDimension();
    }
    all.values = start;
    
    if (compoundDimension_ > dimension_)
	// project to lower dimension
	projection_.project(all.values, projection.values);
    else
	memcpy(projection.values, all.values, sizeof(double) * dimension_);
}

void ompl::base::CompoundProjectionEvaluator::printSettings(std::ostream &out) const
{
    out << "Compound projection [" << std::endl;
    for (unsigned int i = 0 ; i < components_.size() ; ++i)
	components_[i]->printSettings(out);
    out << "]" << std::endl;
}
