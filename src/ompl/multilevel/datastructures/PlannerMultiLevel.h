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

#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_PLANNERMULTILEVEL_
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_PLANNERMULTILEVEL_
#include <ompl/base/Planner.h>

namespace ompl
{
    namespace multilevel
    {
        /** \brief MultiLevel Planner Interface.
         * Extends base::Planner by allowing sequences
         * of base::SpaceInformationPtr */

        class PlannerMultiLevel : public ompl::base::Planner
        {
            using BaseT = ompl::base::Planner;

        public:
            /** \brief Constructor for a set of ompl::base::SpaceInformationPtr
             * which represent different abstraction levels */
            PlannerMultiLevel(std::vector<ompl::base::SpaceInformationPtr> &siVec,
                              std::string type = "PlannerMultiLevel");

            PlannerMultiLevel(ompl::base::SpaceInformationPtr si);

            PlannerMultiLevel(ompl::base::SpaceInformationPtr si, std::string type);

            virtual ~PlannerMultiLevel();

            /** \brief Get ompl::base::ProblemDefinitionPtr for a specific level */
            const ompl::base::ProblemDefinitionPtr &getProblemDefinition(int level) const;

            /** \brief Get ompl::base::ProblemDefinitionPtr for a specific level (non const)*/
            ompl::base::ProblemDefinitionPtr &getProblemDefinitionNonConst(int level);

            /** \brief Get all ompl::base::ProblemDefinitionPtr for all levels
             * in the hierarchy */
            const std::vector<ompl::base::ProblemDefinitionPtr> &getProblemDefinitionVector() const;

            /** \brief Clear multilevel planner by clearing all levels */
            virtual void clear() override;

            /** \brief Number of multilevel abstractions */
            int getLevels() const;

            /** \brief Get dimensionality of the multilevel abstraction */
            std::vector<int> getDimensionsPerLevel() const;

        protected:
            /** \brief Solution paths on each abstraction level */
            std::vector<ompl::base::PathPtr> solutions_;

            /** \brief Sequence of ProblemDefinitionPtr */
            std::vector<ompl::base::ProblemDefinitionPtr> pdefVec_;

            /** \brief Each abstraction level has a unique ompl::base::SpaceInformationPtr */
            std::vector<ompl::base::SpaceInformationPtr> siVec_;
        };
    }  // namespace multilevel
}  // namespace ompl
#endif
