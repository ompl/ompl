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
        OMPL_CLASS_FORWARD(StateSpace);
    }
    namespace multilevel
    {
        OMPL_CLASS_FORWARD(Projection);

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
            virtual void project(
                const ompl::base::State *xBundle, 
                ompl::base::State *xBase) const = 0;

            /* \brief All subclasses need to be able to lift from base space
             * into the total bundle space */
            virtual void lift(
                const ompl::base::State *xBase, 
                ompl::base::State *xBundle) const = 0;

            unsigned int getCoDimension() const;
            unsigned int getDimension() const;
            unsigned int getBaseDimension() const;

            /// \brief Get bundle space
            base::StateSpacePtr getBundle() const;
            /// \brief Get base space
            base::StateSpacePtr getBase() const;

            virtual bool isAdmissible() const;

            /// Type of Bundle Space Projection
            ProjectionType getType() const;
            void setType(const ProjectionType);

            std::string getTypeAsString() const;
            std::string getBundleTypeAsString() const;
            std::string getBaseTypeAsString() const;

            friend std::ostream &operator<<(
                std::ostream &out, 
                const Projection &);
            virtual void print(std::ostream &out) const;
            std::string stateTypeToString(base::StateSpacePtr) const;

        protected:
            base::StateSpacePtr bundleSpace_{nullptr};
            base::StateSpacePtr baseSpace_{nullptr};

            ProjectionType type_;

        };

        class CompoundProjection: public Projection
        {
        public:
            CompoundProjection(
                base::StateSpacePtr bundleSpace, 
                base::StateSpacePtr baseSpace, 
                std::vector<ProjectionPtr>& components);

            virtual ~CompoundProjection() = default;

            /* \brief All subclasses need to be able to project onto base space
             * */
            virtual void project(
                const ompl::base::State *xBundle, 
                ompl::base::State *xBase) const override;

            /* \brief All subclasses need to be able to lift from base space
             * into the total bundle space
             * */
            virtual void lift(
                const ompl::base::State *xBase, 
                ompl::base::State *xBundle) const override;

            /// Dimension of Base Space
            unsigned int getBaseDimension() const;
            /// Dimension of Bundle Space
            unsigned int getDimension() const;
            /// Dimension of Bundle - Dimension of Base
            unsigned int getCoDimension() const;

        private:

            std::vector<ProjectionType> types_;

            std::vector<ProjectionPtr> components_;

        };
    }
}
#endif

