/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage
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

#ifndef OMPL_CONTROL_SPACES_DISCRETE_CONTROL_SPACE_
#define OMPL_CONTROL_SPACES_DISCRETE_CONTROL_SPACE_

#include "ompl/control/ControlSpace.h"

namespace ompl
{
    namespace control
    {
	
        /** \brief Control space sampler for discrete controls */
        class DiscreteControlSampler : public ControlSampler
        {
        public:
	    
            /** \brief Constructor */
            DiscreteControlSampler(const ControlSpace *space) : ControlSampler(space)
            {
            }
	    
            virtual void sample(Control *control);
        };
	
        /** \brief A space representing discrete controls; i.e. there
            are a small number of discrete controls the system can react to.
            Controls are represented as integers [lowerBound, upperBound],
            where lowerBound and upperBound are inclusive. */
        class DiscreteControlSpace : public ControlSpace
        {
        public:
	    
            /** \brief The definition of a discrete control */
            class ControlType : public Control
            {
            public:
		
                /** \brief The current control - an int in range [lowerBound, upperBound] */
                int value;
            };
	    
            /** \brief Construct a discrete space in wich controls can take values in the set [\e lowerBound, \e upperBound] */
            DiscreteControlSpace(const base::StateSpacePtr &stateSpace, int lowerBound, int upperBound) :
		ControlSpace(stateSpace), lowerBound_(lowerBound), upperBound_(upperBound)
            {
                setName("Discrete" + getName());
                type_ = CONTROL_SPACE_DISCRETE;
            }
	    
            virtual ~DiscreteControlSpace(void)
            {
            }
	    
            virtual unsigned int getDimension(void) const;
	    
            virtual void copyControl(Control *destination, const Control *source) const;
	    
            virtual bool equalControls(const Control *control1, const Control *control2) const;
	    
            virtual ControlSamplerPtr allocDefaultControlSampler(void) const;
	    
            virtual Control* allocControl(void) const;
	    
            virtual void freeControl(Control *control) const;
	    
            /** \brief This sets the control value to \e lowerBound_ */
            virtual void nullControl(Control *control) const;
	    
            virtual void printControl(const Control *control, std::ostream &out) const;
	    
            virtual void printSettings(std::ostream &out) const;
	    
            /** \brief Returns the number of controls possible */
            unsigned int getControlCount(void) const
            {
                return upperBound_ - lowerBound_ + 1;
            }
	    
            /** \brief Returns the lowest possible control value */
            int getLowerBound(void) const
            {
                return lowerBound_;
            }
	    
            /** \brief Returns the highest possible control value */
            int getUpperBound(void) const
            {
                return upperBound_;
            }
	    
            /** \brief Set the bounds for the states in this space (the states will be in the set [\e lowerBound, \e upperBound] */
            void setBounds(int lowerBound, int upperBound)
            {
                lowerBound_ = lowerBound;
                upperBound_ = upperBound;
            }
	    
            virtual void setup(void);
	    
        protected:
	    
            /** \brief The lowest integer state */
            int lowerBound_;
	    
            /** \brief The highest integer state */
            int upperBound_;
        };
	
    }
}

#endif
