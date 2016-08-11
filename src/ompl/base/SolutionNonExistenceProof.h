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

/* Author: Ryan Luna */

#ifndef OMPL_BASE_SOLUTION_NON_EXISTENCE_PROOF_
#define OMPL_BASE_SOLUTION_NON_EXISTENCE_PROOF_

#include <utility>

#include "ompl/base/SpaceInformation.h"
#include "ompl/util/ClassForward.h"

namespace ompl
{
    namespace base
    {
        /// @cond IGNORE
        /// \brief Forward declaration of ompl::base::SolutionNonExistenceProof
        OMPL_CLASS_FORWARD(SolutionNonExistenceProof);
        /// @endcond

        /// \class ompl::base::SolutionNonExistenceProofPtr
        /// \brief A shared pointer wrapper for ompl::base::SolutionNonExistenceProof

        /// \brief Abstract definition of a proof for the non-existence of a solution to a problem
        class SolutionNonExistenceProof
        {
        public:
            SolutionNonExistenceProof(SpaceInformationPtr si) : si_(std::move(si))
            {
            }

            virtual ~SolutionNonExistenceProof() = default;

        protected:
            SpaceInformationPtr si_;
        };
    }
}

#endif
