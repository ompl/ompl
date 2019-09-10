/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Rice University
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

/* Author: Mark Moll */

#include "ompl/base/PlannerTerminationCondition.h"

namespace ompl
{
    namespace base
    {
        /** \brief A class to run a planner for a specific number of iterations. Casts to a PTC for use with
         * Planner::solve */
        class IterationTerminationCondition
        {
        public:
            /** \brief Construct a termination condition that can be evaluated numIterations times before returning
             * true. */
            IterationTerminationCondition(unsigned int numIterations);

            /** \brief Increment the number of times eval has been called and check if the planner should now terminate.
             */
            bool eval();

            /** \brief Reset the number of times the IterationTeriminationCondition has been called. */
            void reset();

            /** \brief Cast to a PlannerTerminationCondition */
            operator PlannerTerminationCondition();

            unsigned int getTimesCalled() const
            {
                return timesCalled_;
            }

        private:
            /** \brief The max number of iterations the condition can be called before returning true. */
            unsigned int maxCalls_;
            /** \brief The number of times called so far.*/
            unsigned int timesCalled_;
        };
    }  // namespace base
}  // namespace ompl
