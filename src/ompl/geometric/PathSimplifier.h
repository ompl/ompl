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

#ifndef OMPL_GEOMETRIC_PATH_SIMPLIFIER_
#define OMPL_GEOMETRIC_PATH_SIMPLIFIER_

#include "ompl/base/SpaceInformation.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/util/ClassForward.h"
#include "ompl/util/RandomNumbers.h"
#include <limits>

namespace ompl
{

    namespace geometric
    {

        /** \brief Forward declaration of ompl::geometric::PathSimplifier */
        ClassForward(PathSimplifier);

        /** \class ompl::geometric::PathSimplifierPtr
            \brief A boost shared pointer wrapper for ompl::geometric::PathSimplifier */

        /** \brief This class contains routines that attempt to simplify geometric paths.

            These are in fact routines that shorten the path, and do not
            necessarily make it smoother. */
        class PathSimplifier
        {
        public:

            /** \brief Create an instance for a specified space information */
            PathSimplifier(const base::SpaceInformationPtr &si) : si_(si)
            {
            }

            virtual ~PathSimplifier(void)
            {
            }


            /** \brief Given a path, attempt to remove vertices from
                it while keeping the path valid. This is an iterative
                process that attempts to do "short-cutting" on the
                path.  Connection is attempted between non-consecutive
                states along the path. If the connection is successful,
                the path is shortened by removing the in-between
                states.

                \param path the path to reduce vertices from

                \param maxSteps the maximum number of attempts to
                "short-cut" the path.  If this value is set to 0 (the
                default), the number of attempts made is equal to the
                number of states in \e path.

                \param maxEmptySteps not all iterations of this function
                produce a simplification. If an iteration does not
                produce a simplification, it is called an empty
                step. \e maxEmptySteps denotes the maximum number of
                consecutive empty steps before the simplification
                process terminates. If this value is set to 0 (the
                default), the number of attempts made is equal to the
                number of states in \e path.

                \param rangeRatio the maximum distance between states
                a connection is attempted, as a fraction relative to
                the total number of states (between 0 and 1).

            */
            void reduceVertices(PathGeometric &path, unsigned int maxSteps = 0, unsigned int maxEmptySteps = 0, double rangeRatio = 0.2);

            /** \brief Given a path, attempt to remove vertices from
                it while keeping the path valid. This is an iterative
                process that attempts to do "short-cutting" on the
                path.  Connection is attempted between non-consecutive
                states that are close along the path. If the
                connection is successful, the path is shortened by
                removing the in-between states.

                \param path the path to reduce vertices from

                \param maxSteps the maximum number of attempts to
                "short-cut" the path.  If this value is set to 0 (the
                default), the number of attempts made is equal to the
                number of states in \e path. If this value is set to 0 (the
                default), the number of attempts made is equal to the
                number of states in \e path.

                \param maxEmptySteps not all iterations of this function
                produce a simplification. If an iteration does not
                produce a simplification, it is called an empty
                step. \e maxEmptySteps denotes the maximum number of
                consecutive empty steps before the simplification
                process terminates.
            */
            void collapseCloseVertices(PathGeometric &path, unsigned int maxSteps = 0, unsigned int maxEmptySteps = 0);

            /** \brief Given a path, attempt to smooth it (the
                validity of the path is maintained).

                This function applies \e maxSteps steps of smoothing
                with B-Splines. Fewer steps are applied if no progress
                is detected: states are either not updated or their
                update is smaller than \e minChange.  At each step the
                path is subdivided and states along it are updated
                such that the smoothness is improved.

                \note This function may significantly increase the number of states along the solution path. */
            void smoothBSpline(PathGeometric &path, unsigned int maxSteps = 5, double minChange = std::numeric_limits<double>::epsilon());

            /** \brief Given a path, attempt to remove vertices from
                it while keeping the path valid.  Then, try to smooth
                the path.  */
            virtual void simplifyMax(PathGeometric &path);

        protected:

            /** \brief The space information this path simplifier uses */
            base::SpaceInformationPtr si_;

            /** \brief Instance of random number generator */
            RNG                       rng_;
        };
    }
}

#endif
