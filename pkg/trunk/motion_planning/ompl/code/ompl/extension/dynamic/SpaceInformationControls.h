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

#ifndef OMPL_EXTENSION_DYNAMIC_SPACE_INFORMATION_CONTROLS_
#define OMPL_EXTENSION_DYNAMIC_SPACE_INFORMATION_CONTROLS_

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/StateDistanceEvaluator.h"
#include "ompl/base/Control.h"
#include "ompl/extension/dynamic/PathDynamic.h"
#include "ompl/extension/kinematic/PathKinematic.h"
#include <vector>

/** \brief Main namespace */
namespace ompl
{

    namespace dynamic
    {
	
	/** \brief Space information useful for kinematic planning */
	class SpaceInformationControls : public base::SpaceInformation
	{
	public:
	    
	    /** \brief Constructor; setup() needs to be called as well, before use */
	    SpaceInformationControls(void) : base::SpaceInformation(),
					     m_defaultDistanceEvaluator(dynamic_cast<base::SpaceInformation*>(this))
	    {
		m_stateDistanceEvaluator = &m_defaultDistanceEvaluator;
		m_minControlDuration = m_maxControlDuration = 0;
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

	    
	    /** \brief A class that can perform sampling. Usually an instance of this class is needed
	     * for sampling states or controls */
	    class SamplingCore
	    {	    
	    public:
		SamplingCore(SpaceInformationControls *si) : m_si(si) 
		{
		    m_minControlDuration = m_si->getMinControlDuration();
		    m_maxControlDuration = m_si->getMaxControlDuration();
		}
		
		virtual ~SamplingCore(void)
		{
		}
		
		/** \brief Sample a number of steps */
		virtual unsigned int sampleStepCount(void);
		
		/** \brief Sample a control */
		virtual void sample(base::Control *ctrl);
		
		/** \brief Sample a control near another, within given bounds */
		virtual void sampleNear(base::Control *ctrl, const base::Control *near, const double rho);
		
		/** \brief Sample a control near another, within given bounds */
		virtual void sampleNear(base::Control *ctrl, const base::Control *near, const std::vector<double> &rho);
		
		/** \brief Sample a state */
		virtual void sample(base::State *state);
		
		/** \brief Sample a state near another, within given bounds */
		virtual void sampleNear(base::State *state, const base::State *near, const double rho);
		
		/** \brief Sample a state near another, within given bounds */
		virtual void sampleNear(base::State *state, const base::State *near, const std::vector<double> &rho);
		
	    protected:
		
		SpaceInformationControls *m_si;	    
		random_utils::RNG         m_rng;
		unsigned int              m_minControlDuration;
		unsigned int              m_maxControlDuration;
	    };
	    
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

	    /** \brief Get the semantic description & bounds for a control component */
	    const base::ControlComponent& getControlComponent(unsigned int index) const
	    {
		return m_controlComponent[index];
	    }

	    /** \brief Copy a control to another */
	    virtual void copyControl(base::Control *destination, const base::Control *source) const;

	    /** \brief Make the control a null one */
	    virtual void nullControl(base::Control *ctrl) const;
	    
	    /** \brief Print a state to a stream */
	    void printControl(const base::Control *control, std::ostream &out = std::cout) const;
	    
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
	    std::vector<base::ControlComponent>  m_controlComponent;
	    double                               m_resolution;
	    
	    unsigned int                         m_minControlDuration;
	    unsigned int                         m_maxControlDuration;
	
	    kinematic::PathKinematic            *m_hint;
	    
	private:
	    
	    base::L2SquareStateDistanceEvaluator m_defaultDistanceEvaluator;

	};
    }
    
}

#endif
