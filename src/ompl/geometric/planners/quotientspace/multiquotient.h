/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2019, University of Stuttgart
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*  * Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above
*    copyright notice, this list of conditions and the following
*    disclaimer in the documentation and/or other materials provided
*    with the distribution.
*  * Neither the name of the University of Stuttgart nor the names 
*    of its contributors may be used to endorse or promote products 
*    derived from this software without specific prior written 
*    permission.
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

#ifndef OMPL_GEOMETRIC_PLANNERS_QUOTIENTSPACE_MULTIQUOTIENT_
#define OMPL_GEOMETRIC_PLANNERS_QUOTIENTSPACE_MULTIQUOTIENT_
#include "quotient.h"
#include <type_traits>
#include <queue>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
    namespace geometric
    {
        template <class T, typename Tlast=T>
        class MultiQuotient: public ob::Planner
        {

        static_assert(std::is_base_of<og::Quotient, T>::value, "Template must inherit from Quotient");
        static_assert(std::is_base_of<og::Quotient, Tlast>::value, "Template must inherit from Quotient");

        public:
            const bool DEBUG{false};
            MultiQuotient(std::vector<ob::SpaceInformationPtr> &si_vec, std::string type = "QuotientPlanner");
            void setProblemDefinition(std::vector<ob::ProblemDefinitionPtr> &pdef_vec_);

            virtual ~MultiQuotient() override;

            void getPlannerData(base::PlannerData &data) const override;
            ob::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
            void setup() override;
            void clear() override;
            void setProblemDefinition(const ob::ProblemDefinitionPtr &pdef) override;

            int GetLevels() const;
            std::vector<int> GetFeasibleNodes() const;
            std::vector<int> GetNodes() const;
            std::vector<int> GetDimensionsPerLevel() const;
            void SetStopLevel(uint level_);

        protected:
            std::vector<base::PathPtr> solutions;
            std::vector<og::Quotient*> quotientSpaces;

            bool foundKLevelSolution{false};
            uint currentQuotientLevel{0};
            uint stopAtLevel;

            std::vector<ob::SpaceInformationPtr> si_vec;
            std::vector<ob::ProblemDefinitionPtr> pdef_vec;

            struct CmpQuotientSpacePtrs
            {
                // ">" operator: smallest value is top in queue
                // "<" operator: largest value is top in queue (default)
                bool operator()(const Quotient* lhs, const Quotient* rhs) const
                {
                     return lhs->GetImportance() < rhs->GetImportance();
                }
            };
            typedef std::priority_queue<og::Quotient*, std::vector<og::Quotient*>, CmpQuotientSpacePtrs> QuotientSpacePriorityQueue;
            QuotientSpacePriorityQueue Q;
        };
    }
}
#include "src/multiquotient.ipp"
#endif
