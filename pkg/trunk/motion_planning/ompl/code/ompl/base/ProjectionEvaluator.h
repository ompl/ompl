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

/* \author Ioan Sucan */

#ifndef OMPL_BASE_PROJECTION_EVALUATOR_
#define OMPL_BASE_PROJECTION_EVALUATOR_

#include "ompl/base/General.h"
#include "ompl/base/State.h"
#include <vector>
#include <cmath>

namespace ompl
{
    
    namespace base
    {

	/** Abstract definition for a class computing projections */
	class ProjectionEvaluator
	{
	public:
	    /** Destructor */
	    virtual ~ProjectionEvaluator(void)
	    {
	    }
	    
	    /** Return the dimension of the projection defined by this evaluator */
	    virtual unsigned int getDimension(void) const = 0;
	    
	    /** Compute the projection as an array of double values */
	    virtual void operator()(const State *state, double *projection) const = 0;
	    
	    /** Define the dimension (each component) of a grid cell. The
		number of dimensions set here must be the same as the
		dimension of the projection computed by the projection
		evaluator. */
	    void setCellDimensions(const std::vector<double> &cellDimensions)
	    {
		m_cellDimensions = cellDimensions;
	    }
	    
	    void getCellDimensions(std::vector<double> &cellDimensions) const
	    {
		cellDimensions = m_cellDimensions;
	    }
	    
	    /** Compute integer coordinates for a projection */
	    void computeCoordinates(const double *projection, std::vector<int> &coord) const
	    {
		unsigned int dim = getDimension();
		coord.resize(dim);
		for (unsigned int i = 0 ; i < dim ; ++i)
		    coord[i] = (int)trunc(projection[i]/m_cellDimensions[i]);
	    }
	    
	    void computeCoordinates(const State *state, std::vector<int> &coord) const
	    {
		double projection[getDimension()];
		(*this)(state, projection);
		computeCoordinates(projection, coord);
	    }
	    
	protected:
	    
	    std::vector<double> m_cellDimensions;
	    
	};
    }
}

#endif

