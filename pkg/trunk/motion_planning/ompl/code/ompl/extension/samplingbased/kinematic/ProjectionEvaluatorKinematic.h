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

#ifndef OMPL_EXTENSION_SAMPLINGBASED_KINEMATIC_PROJECTION_EVALUATOR_KINEMATIC_
#define OMPL_EXTENSION_SAMPLINGBASED_KINEMATIC_PROJECTION_EVALUATOR_KINEMATIC_

#include "ompl/extension/samplingbased/kinematic/SpaceInformationKinematic.h"
#include "ompl/base/ProjectionEvaluator.h"

#include <valarray>
#include <vector>

namespace ompl
{
    
    /** Forward class declaration */
    ForwardClassDeclaration(OrthogonalProjectionEvaluator);	
    
    /** Definition for a class computing orthogonal projections */
    class OrthogonalProjectionEvaluator : public ProjectionEvaluator
    {
    public:
	
        OrthogonalProjectionEvaluator(const std::vector<unsigned int> &components) : ProjectionEvaluator()
	{
	    m_components = components;
	}
	
	virtual unsigned int getDimension(void) const
	{
	    return m_components.size();
	}
	
	virtual void operator()(const SpaceInformation::State *state, double *projection) const
	{
	    const SpaceInformationKinematic::StateKinematic *kstate = static_cast<const SpaceInformationKinematic::StateKinematic*>(state);
	    for (unsigned int i = 0 ; i < m_components.size() ; ++i)
		projection[i] = kstate->values[m_components[i]];
	}
	
    protected:
	
	std::vector<unsigned int> m_components;
	
    };	

        /** Definition for a class computing orthogonal projections */
    class LinearProjectionEvaluator : public ProjectionEvaluator
    {
    public:
	
        LinearProjectionEvaluator(const std::vector< std::valarray<double> > &projection) : ProjectionEvaluator()
	{
	    m_projection = projection;
	}
	
	virtual unsigned int getDimension(void) const
	{
	    return m_projection.size();
	}
	
	virtual void operator()(const SpaceInformation::State *state, double *projection) const
	{
	    const SpaceInformationKinematic::StateKinematic *kstate = static_cast<const SpaceInformationKinematic::StateKinematic*>(state);
	    for (unsigned int i = 0 ; i < m_projection.size() ; ++i)
	    {
		const std::valarray<double> &vec = m_projection[i];
		const unsigned int dim = vec.size();
		double *pos = projection + i;
		*pos = 0.0;
		for (unsigned int j = 0 ; j < dim ; ++j)
		    *pos += kstate->values[j] * vec[j];
	    }
	}
	
    protected:
	
	std::vector< std::valarray<double> > m_projection;
	
    };	
    
}

#endif
