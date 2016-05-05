/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Rice University
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

#ifndef OMPL_BASE_PLANNER_STATUS_
#define OMPL_BASE_PLANNER_STATUS_

#include <string>
#include <ostream>

namespace ompl
{
    namespace base
    {
        /// A class to store the exit status of Planner::solve()
        struct PlannerStatus
        {
            /// The possible values of the status returned by a planner
            enum StatusType
            {
                /// Uninitialized status
                UNKNOWN = 0,
                /// Invalid start state or no start state specified
                INVALID_START,
                /// Invalid goal state
                INVALID_GOAL,
                /// The goal is of a type that a planner does not recognize
                UNRECOGNIZED_GOAL_TYPE,
                /// The planner failed to find a solution
                TIMEOUT,
                /// The planner found an approximate solution
                APPROXIMATE_SOLUTION,
                /// The planner found an exact solution
                EXACT_SOLUTION,
                /// The planner crashed
                CRASH,
                /// The planner did not find a solution for some other reason
                ABORT,
                /// The number of possible status values
                TYPE_COUNT
            };

            /// Default constructor
            PlannerStatus(StatusType status = UNKNOWN) : status_(status)
            {
            }

            /// Convenience constructor that sets status_ based on whether some solution was found (\e hasSolution) and
            /// whether that solution was approximate or not (\e isApproximate)
            PlannerStatus(bool hasSolution, bool isApproximate)
              : status_(hasSolution ? (isApproximate ? APPROXIMATE_SOLUTION : EXACT_SOLUTION) : TIMEOUT)
            {
            }

            /// Return a string representation
            std::string asString() const;
            /// Allow casting to true. The value is true iff an approximate or exact solution was found
            operator bool() const
            {
                return status_ == APPROXIMATE_SOLUTION || status_ == EXACT_SOLUTION;
            }
            /// Allow casting to the enum type StatusType
            operator StatusType() const
            {
                return status_;
            }

        private:
            /// Exit status of calling Planner::solve()
            StatusType status_;
        };

        /// Print a PlannerStatus object
        inline std::ostream &operator<<(std::ostream &out, const PlannerStatus &status)
        {
            return out << status.asString();
        }
    }
}

#endif
