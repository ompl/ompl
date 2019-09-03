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

#ifndef OMPL_GEOMETRIC_PLANNERS_QUOTIENTSPACE_SRC_VERTEX_ANNOTATED_
#define OMPL_GEOMETRIC_PLANNERS_QUOTIENTSPACE_SRC_VERTEX_ANNOTATED_

#include <ompl/base/PlannerData.h>
#include <boost/serialization/export.hpp>

namespace ompl
{
    namespace base
    {
        /** \brief An annotated vertex, adding information about its level in the
            quotient-space hiearchy, the maxlevel of quotientspaces and the component it
            belongs to */
        class PlannerDataVertexAnnotated : public ompl::base::PlannerDataVertex
        {
            // If new elements are added,
            // you need to update the clone/getstate functions!
        public:
            PlannerDataVertexAnnotated(const ompl::base::State *st, int tag = 0);
            PlannerDataVertexAnnotated(const PlannerDataVertexAnnotated &rhs);
            virtual PlannerDataVertex *clone() const override;

            void setLevel(unsigned int level_);
            unsigned int getLevel() const;

            void setMaxLevel(unsigned int level_);
            unsigned int getMaxLevel() const;

            void setPath(std::vector<int> path_);
            std::vector<int> getPath() const;

            void setComponent(unsigned int component_);
            unsigned int getComponent() const;

            void setState(ompl::base::State *s);
            void setQuotientState(const ompl::base::State *s);
            virtual const ompl::base::State *getState() const override;
            virtual const ompl::base::State *getQuotientState() const;

            friend bool operator==(const PlannerDataVertexAnnotated &lhs, const PlannerDataVertexAnnotated &rhs);
            friend std::ostream &operator<<(std::ostream &, const PlannerDataVertexAnnotated &);

        protected:
            unsigned int level_{0};
            unsigned int maxLevel_{1};

            std::vector<int> path_;

            unsigned int component_{0};
            const ompl::base::State *stateQuotientSpace_{nullptr};
        };

    }  // namespace base
}  // namespace ompl
#endif
