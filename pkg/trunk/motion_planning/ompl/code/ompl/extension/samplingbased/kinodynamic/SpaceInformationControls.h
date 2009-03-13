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

#ifndef OMPL_EXTENSION_SAMPLINGBASED_KINODYNAMIC_SPACE_INFORMATION_CONTROLS_
#define OMPL_EXTENSION_SAMPLINGBASED_KINODYNAMIC_SPACE_INFORMATION_CONTROLS_

#include "ompl/extension/samplingbased/SpaceInformation.h"
#include "ompl/extension/samplingbased/StateDistanceEvaluator.h"
#include "ompl/extension/samplingbased/kinodynamic/Control.h"
#include <vector>

/** Main namespace */
namespace ompl
{

    namespace sb
    {
	
	/** Space information useful for kinematic planning */
	class SpaceInformationControls : public SpaceInformation
	{
	public:
	    
	    /** Constructor; setup() needs to be called as well, before use */
	    SpaceInformationControls(void) : SpaceInformation(),
					     m_defaultDistanceEvaluator(dynamic_cast<SpaceInformation*>(this))
	    {
		m_stateDistanceEvaluator = &m_defaultDistanceEvaluator;
		m_controlDimension = 0;
		m_resolution = 0.05;
	    }
	    
	    /** Destructor */
	    virtual ~SpaceInformationControls(void)
	    {
	    }

	    
	    /** A class that can perform sampling. Usually an instance of this class is needed
	     * for sampling states */
	    class SamplingCore
	    {	    
	    public:
		SamplingCore(SpaceInformationControls *si) : m_si(si) 
		{
		}	    
		
		virtual ~SamplingCore(void)
		{
		}
		
		/** Sample a number of steps */
		virtual unsigned int sampleStepCount(unsigned int minSteps, unsigned int maxSteps);
		
		/** Sample a control */
		virtual void sample(Control *ctrl);
		
		/** Sample a control near another, within given bounds */
		virtual void sampleNear(Control *ctrl, const Control *near, const double rho);
		
		/** Sample a control near another, within given bounds */
		virtual void sampleNear(Control *ctrl, const Control *near, const std::vector<double> &rho);
		
	    protected:
		
		SpaceInformationControls *m_si;	    
		random_utils::RNG         m_rng;
	    };
	    
	    unsigned int getControlDimension(void) const
	    {
		return m_controlDimension;
	    }
	    
	    const ControlComponent& getControlComponent(unsigned int index) const
	    {
		return m_controlComponent[index];
	    }

	    /** Copy a control to another */
	    virtual void copyControl(Control *destination, const Control *source) const;
	    
	    /** Check if a control is inside the bounding box */
	    bool satisfiesBounds(const Control *control) const;
	    
	    /** Print a state to a stream */
	    void printControl(const Control *control, std::ostream &out = std::cout) const;
	    
	    /** Perform additional tasks to finish the initialization of the space information */
	    virtual void setup(void);
	    
	protected:
	    
	    unsigned int                  m_controlDimension;
	    std::vector<ControlComponent> m_controlComponent;
	    double                        m_resolution;
	    
	private:
	    
	    L2SquareStateDistanceEvaluator m_defaultDistanceEvaluator;

	};
    }
    
}

#endif
