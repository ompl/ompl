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

#ifndef OMPL_BASE_PROJECTION_EVALUATOR_
#define OMPL_BASE_PROJECTION_EVALUATOR_

#include "ompl/base/State.h"
#include "ompl/util/ClassForward.h"
#include <vector>
#include <iostream>
#include <boost/noncopyable.hpp>

namespace ompl
{
    
    namespace base
    {
	
	/** \brief Grid cells corresponding to a projection value are described in terms of their coordinates. */
	typedef std::vector<int> ProjectionCoordinates;

	/** \brief The datatype for state projections. */
	class EuclideanProjection
	{
	public:
	    
	    /** \brief Allocate a projection of dimension n */
	    EuclideanProjection(unsigned int n) : values(new double[n])
	    {	
	    }
	    
	    ~EuclideanProjection(void)
	    {
		delete[] values;
	    }
	    
	    /** \brief Access operator (constant) */
	    double operator[](unsigned int i) const
	    {
		return values[i];	
	    }
	    
	    /** \brief Access operator */
	    double& operator[](unsigned int i)
	    {
		return values[i];	
	    }
	    
	    /** \brief The values of the R<sup>n</sup> vector that makes up the projection */
	    double *values;
	};
	
	ClassForward(StateManifold);

	/** \brief Forward declaration of ompl::base::ProjectionEvaluator */
	ClassForward(ProjectionEvaluator);

	/** \class ompl::base::ProjectionEvaluatorPtr
	    \brief A boost shared pointer wrapper for ompl::base::ProjectionEvaluator */
	
	/** \brief Abstract definition for a class computing
	    projections to R<sup>n</sup>. Implicit integer grids are
	    imposed on this projection space by setting cell
	    sizes. Before use, the user must supply cell dimensions
	    for the integer grid (setCellDimensions()). The
	    implementation of this class is thread safe. */
	class ProjectionEvaluator : private boost::noncopyable
	{
	public:
	    
	    ProjectionEvaluator(const StateManifold *manifold) : manifold_(manifold)
	    {
	    }

	    ProjectionEvaluator(const StateManifoldPtr &manifold) : manifold_(manifold.get())
	    {
	    }
	    
	    virtual ~ProjectionEvaluator(void)
	    {
	    }
	    
	    /** \brief Return the dimension of the projection defined by this evaluator */
	    virtual unsigned int getDimension(void) const = 0;
	    
	    /** \brief Compute the projection as an array of double values */
	    virtual void project(const State *state, EuclideanProjection &projection) const = 0;
	    	    
	    /** \brief Define the dimension (each component) of a grid cell. The
		number of dimensions set here must be the same as the
		dimension of the projection computed by the projection
		evaluator. */
	    void setCellDimensions(const std::vector<double> &cellDimensions);
	    
	    /** \brief Get the dimension (each component) of a grid cell  */
	    const std::vector<double>& getCellDimensions(void) const
	    {
		return cellDimensions_;
	    }
	    
	    /** \brief Check if cell dimensions match projection dimension */
	    void checkCellDimensions(void) const;
	    
	    /** \brief Compute integer coordinates for a projection */
	    void computeCoordinates(const EuclideanProjection &projection, ProjectionCoordinates &coord) const;
	    
	    /** \brief Compute integer coordinates for a state */
	    void computeCoordinates(const State *state, ProjectionCoordinates &coord) const
	    {
		EuclideanProjection projection(getDimension());
		project(state, projection);
		computeCoordinates(projection, coord);
	    }
	    
	    /** \brief Print settings about this projection */
	    virtual void printSettings(std::ostream &out = std::cout) const;

	    /** \brief Print a euclidean projection */
	    virtual void printProjection(const EuclideanProjection &projection, std::ostream &out = std::cout) const;
	    
	protected:
	    
	    /** \brief The manifold this projection operates on */
	    const StateManifold *manifold_;
	    
	    /** \brief The size of a cell, in every dimension of the
		projected space, in the implicitly defined integer
		grid. */
	    std::vector<double>  cellDimensions_;
	    
	};
	
    }
}

#endif

