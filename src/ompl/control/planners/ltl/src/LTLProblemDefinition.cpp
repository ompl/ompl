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

/* Author: Matt Maly */

#include "ompl/control/PathControl.h"
#include "ompl/control/planners/ltl/LTLProblemDefinition.h"
#include "ompl/control/planners/ltl/LTLSpaceInformation.h"
#include "ompl/base/ProblemDefinition.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

oc::LTLProblemDefinition::LTLProblemDefinition(const LTLSpaceInformationPtr &ltlsi)
  : ob::ProblemDefinition(ltlsi), ltlsi_(ltlsi)
{
    createGoal();
}

void oc::LTLProblemDefinition::addLowerStartState(const ob::State *s)
{
    ob::ScopedState<ob::CompoundStateSpace> fullStart(si_);
    ltlsi_->getFullState(s, fullStart.get());
    addStartState(fullStart);
}

ob::PathPtr oc::LTLProblemDefinition::getLowerSolutionPath() const
{
    PathControl *fullPath = static_cast<PathControl *>(getSolutionPath().get());
    auto lowPath(std::make_shared<PathControl>(ltlsi_->getLowSpace()));

    if (fullPath->getStateCount() > 0)
    {
        for (size_t i = 0; i < fullPath->getStateCount() - 1; ++i)
            lowPath->append(ltlsi_->getLowLevelState(fullPath->getState(i)), fullPath->getControl(i),
                            fullPath->getControlDuration(i));

        // The last state does not have a control
        lowPath->append(ltlsi_->getLowLevelState(fullPath->getState(fullPath->getStateCount() - 1)));
    }

    return lowPath;
}

void oc::LTLProblemDefinition::createGoal()
{
    class LTLGoal : public base::Goal
    {
    public:
        LTLGoal(const LTLSpaceInformationPtr &ltlsi) : ob::Goal(ltlsi), ltlsi_(ltlsi), prod_(ltlsi->getProductGraph())
        {
        }
        ~LTLGoal() override = default;
        bool isSatisfied(const ob::State *s) const override
        {
            return prod_->isSolution(ltlsi_->getProdGraphState(s));
        }

    protected:
        const LTLSpaceInformationPtr ltlsi_;
        const ProductGraphPtr prod_;
    };

    // Some compilers have trouble with LTLGoal being hidden in this function,
    // and so we explicitly cast it to its base type.
    setGoal(std::make_shared<LTLGoal>(ltlsi_));
}
