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

#ifndef OMPL_DYNAMIC_SPACE_INFORMATION_CONTROLS_
#define OMPL_DYNAMIC_SPACE_INFORMATION_CONTROLS_

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/L2SquareStateDistanceEvaluator.h"
#include "ompl/dynamic/Control.h"
#include "ompl/dynamic/PathDynamic.h"
#include "ompl/kinematic/PathKinematic.h"
#include "ompl/dynamic/ControlSampler.h"
#include <vector>

namespace ompl
{

    /** \brief The namespace that contains code specific to planning
	under differential constraints */
    namespace dynamic
    {
	
	/** \brief Space information useful for kinematic planning */
	class SpaceInformationControls : public base::SpaceInformation
	{
	public:
	    
	    /** \brief Constructor; setup() needs to be called as well, before use */
	    SpaceInformationControls(void) : base::SpaceInformation(),
					     m_defaultDistanceEvaluator(static_cast<base::SpaceInformation*>(this))
	    {
		m_stateDistanceEvaluator = &m_defaultDistanceEvaluator;
		m_minControlDuration = m_maxControlDuration = 0;
		m_controlSamplerAllocator = NULL;
		m_controlDimension = 0;
		m_resolution = 0.05;
		m_hint = NULL;
	    }
	    
	    /** \brief Destructor */
	    virtual ~SpaceInformationControls(void)
	    {   
		if (m_hint)
		    delete m_hint;
	    }

	    /** \brief Get the resolution (in time) for a control step. The control is applied for time = resolution * control_duration */
	    double getResolution(void) const
	    {
		return m_resolution;
	    }
	    
	    /** \brief Get the control dimension (number of components) */
	    unsigned int getControlDimension(void) const
	    {
		return m_controlDimension;
	    }
	    
	    /** \brief Get the minimum duration for which a control can be applied */
	    unsigned int getMinControlDuration(void) const
	    {
		return m_minControlDuration;
	    }
	    
	    /** \brief Get the maximum duration for which a control can be applied */
	    unsigned int getMaxControlDuration(void) const
	    {
		return m_maxControlDuration;
	    }

	    /** \brief Get the semantic description & bounds for all control components */
	    const std::vector<ControlComponent>& getControlComponents(void) const
	    {
		return m_controlComponent;
	    }

	    /** \brief Get the semantic description & bounds for a control component */
	    const ControlComponent& getControlComponent(unsigned int index) const
	    {
		return m_controlComponent[index];
	    }

	    /** \brief Set the pointer to a function that can allocate
		state samplers.  We do not set a specific instance
		since parallel planners will use multiple instances in
		order to benefit from different random seeds. */
	    void setControlSamplerAllocator(const boost::function<ControlSampler*(const SpaceInformationControls*)> &sampler)
	    {
		m_controlSamplerAllocator = sampler;
	    }	    
	    
	    /** \brief Allocate a new state sampler. Releasing the memory is not handled. */
	    ControlSampler* allocNewControlSampler(void) const
	    {
		return m_controlSamplerAllocator(this);
	    }

	    /** \brief Copy a control to another */
	    virtual void copyControl(Control *destination, const Control *source) const;

	    /** \brief Make the control a null one */
	    virtual void nullControl(Control *ctrl) const;
	    
	    /** \brief Print a state to a stream */
	    void printControl(const Control *control, std::ostream &out = std::cout) const;
	    
	    /** \brief Perform additional tasks to finish the initialization of the space information */
	    virtual void setup(void);
	    
	    /** \brief If a hint is available (a kinematic solution path), provide it here */
	    void setKinematicPath(const kinematic::PathKinematic *hint);
	    
	    /** \brief Get the kinematic solution path provided as hint */
	    const kinematic::PathKinematic* getKinematicPath(void) const
	    {
		return m_hint;
	    }
	    
	protected:
	    
	    unsigned int                         m_controlDimension;
	    std::vector<ControlComponent>        m_controlComponent;
	    double                               m_resolution;
	    
	    unsigned int                         m_minControlDuration;
	    unsigned int                         m_maxControlDuration;
	
	    kinematic::PathKinematic            *m_hint;

	    boost::function<ControlSampler*(const SpaceInformationControls*)>
	                                         m_controlSamplerAllocator;	    
	private:
	    
	    base::L2SquareStateDistanceEvaluator m_defaultDistanceEvaluator;

	};
    }
    
}

#endif
