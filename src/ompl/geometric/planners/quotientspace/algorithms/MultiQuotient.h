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
#include <ompl/geometric/planners/quotientspace/datastructures/QuotientSpace.h>
#include <type_traits>
#include <queue>

namespace ompl
{
    namespace geometric
    {
        /** \brief A sequence of multiple quotient-spaces
             The class MultiQuotient can be used with any planner which inherits
             the ompl::geometric::QuotientSpace class.

             Example usage with QRRT
             (using a sequence si_vec of ompl::base::SpaceInformationPtr)
             ompl::base::PlannerPtr planner =
                 std::make_shared<MultiQuotient<ompl::geometric::QRRT> >(si_vec); */

        template <class T>
        class MultiQuotient : public ompl::base::Planner
        {
            using BaseT = ompl::base::Planner;
            static_assert(std::is_base_of<QuotientSpace, T>::value, "Template must inherit from Quotient");

        public:
            const bool DEBUG{false};

            /** \brief Constructor taking a sequence of ompl::base::SpaceInformationPtr
                 and computing the quotient-spaces for each pair in the sequence */
            MultiQuotient(std::vector<ompl::base::SpaceInformationPtr> &siVec, std::string type = "QuotientPlanner");
            MultiQuotient(ompl::base::SpaceInformationPtr si) = delete;
            MultiQuotient(ompl::base::SpaceInformationPtr si, std::string type) = delete;

            virtual ~MultiQuotient() override;

            /** \brief Return annotated vertices (with information about QuotientSpace level) */
            void getPlannerData(ompl::base::PlannerData &data) const override;

            ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc) override;
            void setup() override;
            void clear() override;
            virtual void setProblemDefinition(const ompl::base::ProblemDefinitionPtr &pdef) override;
            const ompl::base::ProblemDefinitionPtr &getProblemDefinition(unsigned int kQuotientSpace) const;

            /** \brief Number of quotient-spaces */
            int getLevels() const;

            /** \brief Number of feasible nodes on each QuotientSpace (for DEBUGGING) */
            std::vector<int> getFeasibleNodes() const;
            /** \brief Number of nodes on each QuotientSpace (for DEBUGGING) */
            std::vector<int> getNodes() const;

            /** \brief Get all dimensions of the quotient-spaces in the sequence */
            std::vector<int> getDimensionsPerLevel() const;
            void setStopLevel(unsigned int level_);

        protected:
            /** \brief Solution paths on each quotient-space */
            std::vector<ompl::base::PathPtr> solutions_;

            /** \brief Sequence of quotient-spaces */
            std::vector<QuotientSpace *> quotientSpaces_;

            /** \brief Indicator if a solution has been found on the current quotient-spaces */
            bool foundKLevelSolution_{false};

            /** \brief Current level on which we have not yet found a path */
            unsigned int currentQuotientLevel_{0};

            /** \brief \brief Sometimes we only want to plan until a certain quotient-space
                level (for debugging for example). This variable sets the stopping
                level. */
            unsigned int stopAtLevel_;

            /** \brief Each QuotientSpace has a unique ompl::base::SpaceInformationPtr */
            std::vector<ompl::base::SpaceInformationPtr> siVec_;

            /** \brief Compare function for priority queue */
            struct CmpQuotientSpacePtrs
            {
                // ">" operator: smallest value is top in queue
                // "<" operator: largest value is top in queue (default)
                bool operator()(const QuotientSpace *lhs, const QuotientSpace *rhs) const
                {
                    return lhs->getImportance() < rhs->getImportance();
                }
            };
            /** \brief \brief Priority queue of QuotientSpaces which keeps track of how often
                every tree on each space has been expanded. */
            typedef std::priority_queue<QuotientSpace *, std::vector<QuotientSpace *>, CmpQuotientSpacePtrs>
                QuotientSpacePriorityQueue;
            QuotientSpacePriorityQueue priorityQueue_;
        };
    }  // namespace geometric
}  // namespace ompl
#include <ompl/geometric/planners/quotientspace/algorithms/MultiQuotientImpl.h>
#endif
