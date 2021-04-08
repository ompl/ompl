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

#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_BUNDLESPACESEQUENCE_
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_BUNDLESPACESEQUENCE_
#include <ompl/multilevel/datastructures/BundleSpace.h>
#include <ompl/multilevel/datastructures/PlannerMultiLevel.h>
#include <ompl/multilevel/datastructures/pathrestriction/FindSectionTypes.h>
#include <type_traits>
#include <queue>

namespace ompl
{
    namespace base
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::GoalState */
        OMPL_CLASS_FORWARD(GoalState);
        /** \brief Forward declaration of ompl::base::GoalStates */
        OMPL_CLASS_FORWARD(GoalStates);
        /// @endcond
    }
    namespace multilevel
    {
        /** \brief A planner for a sequence of BundleSpaces

             Example usage with QRRT
             ompl::base::PlannerPtr planner =
                 std::make_shared<BundleSpaceSequence<ompl::multilevel::QRRT> >(si_vec);

             whereby si_vec is of type std::vector<ompl::base::SpaceInformationPtr>

             \tparam T Planner function used to grow each space
         */
        template <class T>
        class BundleSpaceSequence : public PlannerMultiLevel
        {
            using BaseT = ompl::multilevel::PlannerMultiLevel;
            static_assert(std::is_base_of<BundleSpace, T>::value, "Template must inherit from BundleSpace");

        public:
            /** \brief Non-multilevel Mode: Calling with a single
             * ompl::base::SpaceInformationPtr will revert to standard planning
             * without invoking any projection operators. */
            BundleSpaceSequence(ompl::base::SpaceInformationPtr si,
                                std::string type = "BundleSpacePlannerNonMultilevel");

            /** \brief Basic Mode: Specify vector of ompl::base::SpaceInformationPtr
                 and let the algorithm figure out the projections itself.*/
            BundleSpaceSequence(std::vector<ompl::base::SpaceInformationPtr> &siVec,
                                std::string type = "BundleSpacePlanner");

            /** \brief Advanced Mode: Specify not only the vector of
             * ompl::base::SpaceInformationPtr, but also how
             * we should project from each bundle space to each base space. */
            BundleSpaceSequence(std::vector<ompl::base::SpaceInformationPtr> &siVec,
                                std::vector<ompl::multilevel::ProjectionPtr> &projVec,
                                std::string type = "BundleSpacePlannerCustomProjection");

            virtual ~BundleSpaceSequence();

            void declareBundleSpaces(bool guessProjection = true);

            /** \brief Return annotated vertices (with information about BundleSpace level) */
            virtual void getPlannerData(ompl::base::PlannerData &data) const override;

            ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc) override;

            virtual void setup() override;
            virtual void clear() override;
            virtual void setProblemDefinition(const ompl::base::ProblemDefinitionPtr &pdef) override;

            void setStopLevel(unsigned int level_);

            /** \brief Set strategy to use to solve the find section problem.
             * */
            void setFindSectionStrategy(FindSectionType type);

        protected:
            /** \brief Starting from a baseState on baseLevel, we lift
             * it iteratively upwards into the total space of the sequence. For
             * each lift, we choose an identity fiber element using the
             * allocIdentityState method in ompl::multilevel::BundleSpace. */
            ompl::base::State *getTotalState(int baseLevel, const base::State *baseState) const;

            /** \brief Sequence of BundleSpaces */
            std::vector<T *> bundleSpaces_;

            /** \brief Indicator if a solution has been found on the current BundleSpaces */
            bool foundKLevelSolution_{false};

            /** \brief Current level on which we have not yet found a path */
            unsigned int currentBundleSpaceLevel_{0};

            /** \brief \brief Sometimes we only want to plan until a certain BundleSpace
                level (for debugging for example). This variable sets the stopping
                level. */
            unsigned int stopAtLevel_;

            /** \brief Compare function for priority queue */
            struct CmpBundleSpacePtrs
            {
                // ">" operator: smallest value is top in queue
                // "<" operator: largest value is top in queue (default)
                bool operator()(const BundleSpace *lhs, const BundleSpace *rhs) const
                {
                    return lhs->getImportance() < rhs->getImportance();
                }
            };
            /** \brief \brief Priority queue of BundleSpaces which keeps track of how often
                every graph on each space has been expanded. */
            typedef std::priority_queue<BundleSpace *, std::vector<BundleSpace *>, CmpBundleSpacePtrs>
                BundleSpacePriorityQueue;
            BundleSpacePriorityQueue priorityQueue_;
        };
    }  // namespace multilevel
}  // namespace ompl
#include "BundleSpaceSequenceImpl.h"
#endif
