/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Rice University
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

/* Author: Matt Maly */

#ifndef OMPL_CONTROL_PLANNERS_LTL_LTLPROBLEMDEFINITION_
#define OMPL_CONTROL_PLANNERS_LTL_LTLPROBLEMDEFINITION_

#include "ompl/base/ProblemDefinition.h"
#include "ompl/control/planners/ltl/ProductGraph.h"
#include "ompl/control/planners/ltl/LTLSpaceInformation.h"

namespace ompl
{
    namespace control
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::control::LTLProblemDefinition */
        OMPL_CLASS_FORWARD(LTLProblemDefinition);
        /// @endcond

        /** \class ompl::control::LTLProblemDefinitionPtr
            \brief A shared pointer wrapper for ompl::control::LTLProblemDefinition */
        class LTLProblemDefinition : public base::ProblemDefinition
        {
        public:
            LTLProblemDefinition(const control::LTLSpaceInformationPtr &ltlsi);

            ~LTLProblemDefinition() override = default;

            void addLowerStartState(const base::State *s);

            base::PathPtr getLowerSolutionPath() const;

        protected:
            void createGoal();

            LTLSpaceInformationPtr ltlsi_;
        };
    }
}

#endif
