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

#ifndef OMPL_BASE_PATH_
#define OMPL_BASE_PATH_

#include "ompl/util/ClassForward.h"
#include "ompl/base/Cost.h"
#include <iostream>
#include <boost/concept_check.hpp>
#include <utility>

namespace ompl
{
    namespace base
    {
        /// @cond IGNORE
        OMPL_CLASS_FORWARD(SpaceInformation);
        /// @endcond

        /// @cond IGNORE
        OMPL_CLASS_FORWARD(OptimizationObjective);
        /// @endcond

        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::Path */
        OMPL_CLASS_FORWARD(Path);
        /// @endcond

        /** \class ompl::base::PathPtr
            \brief A shared pointer wrapper for ompl::base::Path */

        /** \brief Abstract definition of a path */
        class Path
        {
        public:
            // non-copyable
            Path(const Path &) = delete;
            Path &operator=(const Path &) = delete;

            /** \brief Constructor. A path must always know the space information it is part of */
            Path(SpaceInformationPtr si) : si_(std::move(si))
            {
            }

            /** \brief Destructor */
            virtual ~Path() = default;

            /** \brief Get the space information associated to this class */
            const SpaceInformationPtr &getSpaceInformation() const
            {
                return si_;
            }

            /** \brief Cast this instance to a desired type. */
            template <class T>
            const T *as() const
            {
                /** \brief Make sure the type we are allocating is indeed a Path */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T *, Path *>));

                return static_cast<const T *>(this);
            }

            /** \brief Cast this instance to a desired type. */
            template <class T>
            T *as()
            {
                /** \brief Make sure the type we are allocating is indeed a Path */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T *, Path *>));

                return static_cast<T *>(this);
            }

            /** \brief Return the length of a path */
            virtual double length() const = 0;

            /** \brief Return the cost of the path with respect to a
                specified optimization objective. */
            virtual Cost cost(const OptimizationObjectivePtr &obj) const = 0;

            /** \brief Check if the path is valid */
            virtual bool check() const = 0;

            /** \brief Print the path to a stream */
            virtual void print(std::ostream &out) const = 0;

        protected:
            /** \brief The space information this path is part of */
            SpaceInformationPtr si_;
        };
    }
}

#endif
