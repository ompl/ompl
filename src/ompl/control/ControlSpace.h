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

#ifndef OMPL_CONTROL_CONTROL_SPACE_
#define OMPL_CONTROL_CONTROL_SPACE_

#include "ompl/base/StateSpace.h"
#include "ompl/control/Control.h"
#include "ompl/control/ControlSampler.h"
#include "ompl/control/ControlSpaceTypes.h"
#include "ompl/util/Console.h"
#include "ompl/util/ClassForward.h"
#include <boost/concept_check.hpp>
#include <boost/noncopyable.hpp>
#include <iostream>
#include <vector>

namespace ompl
{

    namespace control
    {

        /// @cond IGNORE
        /** \brief Forward declaration of ompl::control::ControlSpace */
        ClassForward(ControlSpace);
        /// @endcond

        /** \class ompl::control::ControlSpacePtr
            \brief A boost shared pointer wrapper for ompl::control::ControlSpace */

        /** \brief A control space representing the space of applicable controls */
        class ControlSpace : private boost::noncopyable
        {
        public:

            /** \brief Construct a control space, given the state space */
            ControlSpace(const base::StateSpacePtr &stateSpace);

            virtual ~ControlSpace(void);

            /** \brief Cast this instance to a desired type. */
            template<class T>
            T* as(void)
            {
                /** \brief Make sure the type we are casting to is indeed a control space */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T*, ControlSpace*>));

                return static_cast<T*>(this);
            }

            /** \brief Cast this instance to a desired type. */
            template<class T>
            const T* as(void) const
            {
                /** \brief Make sure the type we are casting to is indeed a control space */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T*, ControlSpace*>));

                return static_cast<const T*>(this);
            }

            /** \brief Get the name of the control space */
            const std::string& getName(void) const;

            /** \brief Set the name of the control space */
            void setName(const std::string &name);

            /** \brief Get the type of the control space. The type can be
                used to verify whether two space instances are of
                the same type */
            int getType(void) const
            {
                return type_;
            }

            /** \brief Return the state space this control space depends on */
            const base::StateSpacePtr& getStateSpace(void) const
            {
                return stateSpace_;
            }

            /** \brief Get the dimension of this control space */
            virtual unsigned int getDimension(void) const = 0;

            /** \brief Allocate memory for a control */
            virtual Control* allocControl(void) const = 0;

            /** \brief Free the memory of a control */
            virtual void freeControl(Control *control) const = 0;

            /** \brief Copy a control to another */
            virtual void copyControl(Control *destination, const Control *source) const = 0;

            /** \brief Check if two controls are the same */
            virtual bool equalControls(const Control *control1, const Control *control2) const = 0;

            /** \brief Make the control have no effect if it were to be applied to a state for any amount of time. */
            virtual void nullControl(Control *control) const = 0;

            /** \brief Allocate the default control sampler */
            virtual ControlSamplerPtr allocDefaultControlSampler(void) const = 0;

            /** \brief Allocate an instance of the control sampler for this space. This sampler will be allocated with the
                sampler allocator that was previously specified by setControlSamplerAllocator() or, if no sampler allocator was specified,
                allocDefaultControlSampler() is called */
            virtual ControlSamplerPtr allocControlSampler(void) const;

            /** \brief Set the sampler allocator to use */
            void setControlSamplerAllocator(const ControlSamplerAllocator &csa);

            /** \brief Clear the control sampler allocator (reset to default) */
            void clearControlSamplerAllocator(void);

            /** \brief Many controls contain a number of double values. This function provides a means to get the
                memory address of a double value from a control \e control located at position \e index. The first double value
                is returned for \e index = 0. If \e index is too large (does not point to any double values in the control),
                the return value is NULL. */
            virtual double* getValueAddressAtIndex(Control *control, const unsigned int index) const;

            /** \brief Print a control to a stream */
            virtual void printControl(const Control *control, std::ostream &out) const;

            /** \brief Print the settings for this control space to a stream */
            virtual void printSettings(std::ostream &out) const;

            /** \brief Perform final setup steps. This function is automatically called by the SpaceInformation */
            virtual void setup(void);

        protected:

            /** \brief A type assigned for this control space */
            int                     type_;

            /** \brief The state space controls can be applied to */
            base::StateSpacePtr     stateSpace_;

            /** \brief An optional control sampler allocator */
            ControlSamplerAllocator csa_;

        private:

            /** \brief The name of this control space */
            std::string             name_;
        };

        /** \brief A control space to allow the composition of control spaces */
        class CompoundControlSpace : public ControlSpace
        {
        public:

            /** \brief Define the type of control allocated by this control space */
            typedef CompoundControl ControlType;

            /** \brief Constructor. The corresponding state space needs to be specified. */
            CompoundControlSpace(const base::StateSpacePtr &stateSpace) : ControlSpace(stateSpace), componentCount_(0), locked_(false)
            {
            }

            virtual ~CompoundControlSpace(void)
            {
            }

            /** \brief Cast a component of this instance to a desired type. */
            template<class T>
            T* as(const unsigned int index) const
            {
                /** \brief Make sure the type we are casting to is indeed a control space */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T*, ControlSpace*>));

                return static_cast<T*>(getSubSpace(index).get());
            }

            /** \brief Adds a control space as a component of the compound control space. */
            virtual void addSubSpace(const ControlSpacePtr &component);

            /** \brief Get the number of control spaces that make up the compound control space */
            unsigned int getSubSpaceCount(void) const;

            /** \brief Get a specific subspace from the compound control space */
            const ControlSpacePtr& getSubSpace(const unsigned int index) const;

            /** \brief Get a specific subspace from the compound control space */
            const ControlSpacePtr& getSubSpace(const std::string &name) const;

            virtual unsigned int getDimension(void) const;

            virtual Control* allocControl(void) const;

            virtual void freeControl(Control *control) const;

            virtual void copyControl(Control *destination, const Control *source) const;

            virtual bool equalControls(const Control *control1, const Control *control2) const;

            virtual void nullControl(Control *control) const;

            virtual ControlSamplerPtr allocDefaultControlSampler(void) const;

            virtual double* getValueAddressAtIndex(Control *control, const unsigned int index) const;

            virtual void printControl(const Control *control, std::ostream &out = std::cout) const;

            virtual void printSettings(std::ostream &out) const;

            virtual void setup(void);

            /** \brief Lock this control space. This means no further
             control spaces can be added as components.  This function can
             be for instance called from the constructor of a state space
             that inherits from CompoundControlSpace to prevent the
             user to add further components. */
            void lock(void);

        protected:

            /** \brief The component control spaces that make up the compound control space */
            std::vector<ControlSpacePtr>    components_;

            /** \brief The number of contained components */
            unsigned int                    componentCount_;

            /** \brief Flag indicating whether adding further components is allowed or not */
            bool                            locked_;
        };
    }
}

#endif
