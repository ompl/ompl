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
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the University of Stuttgart nor the names
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

#ifndef OMPL_GEOMETRIC_PLANNERS_QUOTIENTSPACE_MULTIQUOTIENT_
#define OMPL_GEOMETRIC_PLANNERS_QUOTIENTSPACE_MULTIQUOTIENT_
#include "QuotientSpace.h"
#include <type_traits>
#include <queue>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
    namespace geometric
    {
        /** \brief A sequence of multiple quotient-spaces
             The class MultiQuotient can be used with any planner which inherits
             the og::QuotientSpace class.

             Example usage with QRRT
             (using a sequence si_vec of ob::SpaceInformationPtr)
               ob::PlannerPtr planner =
                 std::make_shared<MultiQuotient<og::QRRT> >(si_vec); */

        template <class T>
        class MultiQuotient : public ob::Planner
        {
            static_assert(std::is_base_of<og::QuotientSpace, T>::value, "Template must inherit from Quotient");

        public:
            const bool DEBUG{false};

            /// \brief Constructor taking a sequence of ob::SpaceInformationPtr
            ///  and computing the quotient-spaces for each pair in the sequence
            MultiQuotient(std::vector<ob::SpaceInformationPtr> &si_vec, std::string type = "QuotientPlanner");
            /** \brief Sequence of Problemdefinition, containing projected start and
                goal configuration for each ob::SpaceInformationPtr
                 Example usage:
                std::static_pointer_cast<MultiQuotient<og::QRRT>>(planner)
                          ->setProblemDefinition(pdef_vec); */

            void setProblemDefinition(std::vector<ob::ProblemDefinitionPtr> &pdef_vec_);

            virtual ~MultiQuotient() override;

            /// Return annotated vertices (with information about QuotientSpace level)
            void getPlannerData(ob::PlannerData &data) const override;

            ob::PlannerStatus solve(const ob::PlannerTerminationCondition &ptc) override;
            void setup() override;
            void clear() override;
            void setProblemDefinition(const ob::ProblemDefinitionPtr &pdef) override;

            /// Number of quotient-spaces
            int getLevels() const;

            /// Number of feasible nodes on each QuotientSpace (for DEBUGGING)
            std::vector<int> getFeasibleNodes() const;
            /// Number of nodes on each QuotientSpace (for DEBUGGING)
            std::vector<int> getNodes() const;

            /// Get all dimensions of the quotient-spaces in the sequence
            std::vector<int> getDimensionsPerLevel() const;
            void setStopLevel(unsigned int level_);

        protected:
            /// Solution paths on each quotient-space
            std::vector<ob::PathPtr> solutions_;

            /// Sequence of quotient-spaces
            std::vector<og::QuotientSpace *> quotientSpaces_;

            /// Indicator if a solution has been found on the current quotient-spaces
            bool foundKLevelSolution_{false};

            /// Current level on which we have not yet found a path
            unsigned int currentQuotientLevel_{0};

            /// \brief Sometimes we only want to plan until a certain quotient-space
            /// level (for debugging for example). This variable sets the stopping
            /// level.
            unsigned int stopAtLevel_;

            /// Each QuotientSpace has a unique ob::SpaceInformationPtr 
            std::vector<ob::SpaceInformationPtr> siVec_;

            /// \brief Each QuotientSpace has a ProblemDefinition, which contains the
            /// projected start and goal configurations. After planning each
            /// ProblemDefinitionPtr also contains the solution path on each QuotientSpace.
            std::vector<ob::ProblemDefinitionPtr> pdefVec_;

            /// Compare function for priority queue
            struct CmpQuotientSpacePtrs
            {
                // ">" operator: smallest value is top in queue
                // "<" operator: largest value is top in queue (default)
                bool operator()(const QuotientSpace *lhs, const QuotientSpace *rhs) const
                {
                    return lhs->getImportance() < rhs->getImportance();
                }
            };
            /// \brief Priority queue of QuotientSpaces which keeps track of how often
            /// every tree on each space has been expanded.
            typedef std::priority_queue<og::QuotientSpace *, std::vector<og::QuotientSpace *>, CmpQuotientSpacePtrs>
                QuotientSpacePriorityQueue;
            QuotientSpacePriorityQueue priorityQueue_;
        };
    }
}
#include "MultiQuotient.ipp"
#endif
