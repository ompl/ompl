/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, 
 *  Max Planck Institute for Intelligent Systems (MPI-IS).
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
 *   * Neither the name of the MPI-IS nor the names
 *     of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written
 *     permission.
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

/* Author: Andreas Orthey */

#include <ompl/multilevel/datastructures/PlannerMultiLevel.h>

ompl::multilevel::PlannerMultiLevel::PlannerMultiLevel(
    std::vector<ompl::base::SpaceInformationPtr> &siVec, std::string type)
  : BaseT(siVec.back(), type), siVec_(siVec)
{
}

ompl::multilevel::PlannerMultiLevel::~PlannerMultiLevel()
{
}

void ompl::multilevel::PlannerMultiLevel::clear()
{
    BaseT::clear();
    solutions_.clear();
    pdef_->clearSolutionPaths();
    for(uint k = 0; k < pdefVec_.size(); k++)
    {
        pdefVec_.at(k)->clearSolutionPaths();
    }
}


std::vector<int> ompl::multilevel::PlannerMultiLevel::getDimensionsPerLevel() const
{
    std::vector<int> dimensionsPerLevel;
    for (unsigned int k = 0; k < siVec_.size(); k++)
    {
        unsigned int Nk = siVec_.at(k)->getStateDimension();
        dimensionsPerLevel.push_back(Nk);
    }
    return dimensionsPerLevel;
}

int ompl::multilevel::PlannerMultiLevel::getLevels() const
{
    return siVec_.size();
}

const ompl::base::ProblemDefinitionPtr &
ompl::multilevel::PlannerMultiLevel::getProblemDefinition(int level) const
{
    return pdefVec_.at(level);
}

const std::vector<ompl::base::ProblemDefinitionPtr>&
ompl::multilevel::PlannerMultiLevel::getProblemDefinitionVector() const
{
    return pdefVec_;
}
