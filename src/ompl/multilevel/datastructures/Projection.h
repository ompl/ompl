/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021,
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

#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_BUNDLE_PROJECTION_
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_BUNDLE_PROJECTION_
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/multilevel/datastructures/ProjectionTypes.h>

namespace ompl
{
    namespace base
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::StateSpace */
        OMPL_CLASS_FORWARD(StateSpace);
        /// @endcond
    }
    namespace multilevel
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::multilevel::Projection */
        OMPL_CLASS_FORWARD(Projection);
        /// @endcond

        /* \brief A projection which consists of a set of component projections
         * */
        class Projection
        {
        public:
            Projection() = delete;
            Projection(base::StateSpacePtr bundleSpace, base::StateSpacePtr baseSpace);

            virtual ~Projection() = default;

            /* \brief All subclasses need to be able to project onto base space
             * */
            virtual void project(const ompl::base::State *xBundle, ompl::base::State *xBase) const = 0;

            /* \brief All subclasses need to be able to lift from base space
             * into the total bundle space */
            virtual void lift(const ompl::base::State *xBase, ompl::base::State *xBundle) const = 0;

            /* \brief Check if projection is fibered (has explicit fiber space) */
            virtual bool isFibered() const;

            /* \brief Co-dimension of projection (dimension of null space) */
            unsigned int getCoDimension() const;

            /* \brief Dimension of bundle space of projection */
            unsigned int getDimension() const;

            /* \brief Dimension of base space */
            unsigned int getBaseDimension() const;

            /* \brief Get bundle space */
            base::StateSpacePtr getBundle() const;

            /* \brief Get base space */
            base::StateSpacePtr getBase() const;

            /* \brief Check if projection is admissible (NYI) */
            virtual bool isAdmissible() const;

            /* \brief Type of Bundle Space Projection */
            ProjectionType getType() const;
            /* \brief Set type of Bundle Space Projection */
            void setType(const ProjectionType);

            /* \brief Projection type to std::string */
            std::string getTypeAsString() const;
            /* \brief Bundle space as std::string */
            std::string getBundleTypeAsString() const;
            /* \brief Base space as std::string */
            std::string getBaseTypeAsString() const;

            /// Print to stream (actual implementation in print(std::ostream &out))
            friend std::ostream &operator<<(std::ostream &out, const Projection &);

            /// Print to stream
            virtual void print(std::ostream &out) const;

            /// Return string representing type of ompl::base::StateSpace
            std::string stateTypeToString(base::StateSpacePtr) const;

        protected:
            base::StateSpacePtr bundleSpace_{nullptr};
            base::StateSpacePtr baseSpace_{nullptr};

            ProjectionType type_;
        };

        class CompoundProjection : public Projection
        {
        public:
            CompoundProjection(const base::StateSpacePtr &bundleSpace, const base::StateSpacePtr &baseSpace,
                               const std::vector<ProjectionPtr> &components);

            virtual ~CompoundProjection() = default;

            /* \brief All subclasses need to be able to project onto base space
             * */
            void project(const ompl::base::State *xBundle, ompl::base::State *xBase) const override;

            /* \brief All subclasses need to be able to lift from base space
             * into the total bundle space
             * */
            void lift(const ompl::base::State *xBase, ompl::base::State *xBundle) const override;
            /// Print to stream
            virtual void print(std::ostream &out) const override;

            /// Dimension of Base Space
            unsigned int getBaseDimension() const;
            /// Dimension of Bundle Space
            unsigned int getDimension() const;
            /// Dimension of Bundle - Dimension of Base
            unsigned int getCoDimension() const;

            /// Check that every compound has an explicit fiber representation
            bool isFibered() const override;

        private:
            std::vector<ProjectionType> types_;

            std::vector<ProjectionPtr> components_;
        };
    }
}
#endif
