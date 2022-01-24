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

#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_SRC_VERTEX_ANNOTATED_
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_SRC_VERTEX_ANNOTATED_

#include <ompl/base/PlannerData.h>
#include <boost/serialization/export.hpp>

namespace ompl
{
    namespace multilevel
    {
        /** \brief An annotated vertex, adding information about its level in
            the multilevel hierarchy. Class has two modes: Mode 1 (baseMode), we
            store a reference to its base state element. In Mode 2 (totalMode),
            we store a deep copy of the lift of the base state into the total
            space (NOTE: required for PlannerData functions like decoupleFromPlanner())
        */

        class PlannerDataVertexAnnotated : public ompl::base::PlannerDataVertex
        {
            // If new elements are added,
            // you need to update the clone/getstate functions!
        public:
            /** \brief Constructor for base state. Set mode to baseMode. */
            PlannerDataVertexAnnotated(const ompl::base::State *state);

            PlannerDataVertexAnnotated(const PlannerDataVertexAnnotated &rhs);
            virtual ~PlannerDataVertexAnnotated() override;
            virtual PlannerDataVertex *clone() const override;

            /** \brief The level of vertex in the bundle space hierarchy */
            void setLevel(unsigned int level_);
            unsigned int getLevel() const;

            /** \brief The maximum level in the bundle space hierarchy */
            void setMaxLevel(unsigned int level_);
            unsigned int getMaxLevel() const;

            /** \brief The component of vertex in the graph (start, goal or
             * other component) */
            void setComponent(unsigned int component_);
            unsigned int getComponent() const;

            /** \brief Set total state, i.e. the lift of the base state to the
               total space (last Spaceinformationptr in sequence).
               NOTE: Changes mode to totalMode.
               NOTE: requires Spaceinformationptr (of total space)
               to free state
            */
            void setTotalState(ompl::base::State *s, ompl::base::SpaceInformationPtr si);

            /** \brief Explicitly changes base state (does not change mode) */
            void setBaseState(const ompl::base::State *s);

            /** \brief Returns base state in baseMode and total state in
             * totalMode. The total space here is the last element of space sequence.
             * */
            virtual const ompl::base::State *getState() const override;

            /** \brief Same as getState(), but state can be changed */
            ompl::base::State *getStateNonConst() const;

            /** \brief Returns base state, indepent of mode*/
            const ompl::base::State *getBaseState() const;

            ompl::base::SpaceInformationPtr getSpaceInformationPtr() const;

            /** \brief Verifies equality by checking level and base state (mode
             * independent) */
            virtual bool operator==(const PlannerDataVertex &rhs) const override;

            friend std::ostream &operator<<(std::ostream &, const PlannerDataVertexAnnotated &);

        protected:
            /** \brief The level for the base state */
            unsigned int level_{0};

            /** \brief How many spaces exists in the multilevel structure */
            unsigned int maxLevel_{1};

            /** \brief (Optional:) which component in roadmap does vertex belong
             * to */
            unsigned int component_{0};

            /** \brief There are two modes. Mode 1 is the normal mode where this
             * class contains a reference to the base state. In
             * that case getState() returns the base state. Mode 2 is the total
             * space mode, where we set a total
             * state, in which case getState() returns the total state. Note
             * that we require the class to be in Mode 2 for methods like
             * PlannerData::decoupleFromPlanner(). You can put the class into
             * Mode 2 by calling setTotalState()
             */
            bool totalStateIsSet{false};

            /** \brief Internal reference to base state. Same as state_ in
             * normal Mode to avoid confusion. */
            const ompl::base::State *stateBase_{nullptr};

            /** \brief Storage of total state */
            ompl::base::State *stateTotal_{nullptr};

            /** \brief Pointer to total space (to free total space element upon
             * deletion) */
            ompl::base::SpaceInformationPtr si_{nullptr};
        };

    }  // namespace multilevel
}  // namespace ompl
#endif
