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
#include <cmath>

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

	    
