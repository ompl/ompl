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

#ifndef OMPL_CONTROL_CONTROL_
#define OMPL_CONTROL_CONTROL_

#include <boost/concept_check.hpp>

namespace ompl
{
    namespace control
    {
        /** \brief Definition of an abstract control */
        class Control
        {
        private:
            /** \brief Disable copy-constructor */
            Control(const Control &);

            /** \brief Disable copy operator */
            const Control &operator=(const Control &);

        protected:
            Control()
            {
            }

            virtual ~Control()
            {
            }

        public:
            /** \brief Cast this instance to a desired type. */
            template <class T>
            const T *as() const
            {
                /** \brief Make sure the type we are allocating is indeed a state */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T *, Control *>));

                return static_cast<const T *>(this);
            }

            /** \brief Cast this instance to a desired type. */
            template <class T>
            T *as()
            {
                /** \brief Make sure the type we are allocating is indeed a state */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T *, Control *>));

                return static_cast<T *>(this);
            }
        };

        /** \brief Definition of a compound control */
        class CompoundControl : public Control
        {
        public:
            CompoundControl() : Control()
            {
            }

            virtual ~CompoundControl()
            {
            }

            /** \brief Cast a component of this instance to a desired type. */
            template <class T>
            const T *as(const unsigned int index) const
            {
                /** \brief Make sure the type we are allocating is indeed a state */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T *, Control *>));

                return static_cast<const T *>(components[index]);
            }

            /** \brief Cast a component of this instance to a desired type. */
            template <class T>
            T *as(const unsigned int index)
            {
                /** \brief Make sure the type we are allocating is indeed a state */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T *, Control *>));

                return static_cast<T *>(components[index]);
            }

            /** \brief The components that make up a compound control */
            Control **components;
        };
    }
}
#endif
