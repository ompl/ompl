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

#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_BUNDLE_COMPONENT_
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_BUNDLE_COMPONENT_
#include <ompl/base/State.h>
#include <ompl/base/StateSpaceTypes.h>
#include <ompl/multilevel/datastructures/ProjectionTypes.h>
#include <ompl/multilevel/datastructures/Projection.h>

namespace ompl
{
    namespace base
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::SpaceInformation */
        OMPL_CLASS_FORWARD(SpaceInformation);
        /** \brief Forward declaration of ompl::base::StateSpace */
        OMPL_CLASS_FORWARD(StateSpace);
        /// @endcond
    }
    namespace multilevel
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::multilevel::FiberedProjection */
        OMPL_CLASS_FORWARD(FiberedProjection);
        /// @endcond

        /* \brief A bundle projection with an explicit fiber space representation
         * which can be explicitly sampled to lift states */
        class FiberedProjection : public Projection
        {
            friend class CompoundFiberedProjection;
        public:
            FiberedProjection(base::StateSpacePtr bundleSpace, base::StateSpacePtr baseSpace);

            virtual ~FiberedProjection() = default;

            /* \brief Lift state from base to bundle */
            virtual void lift(const ompl::base::State *xBase, ompl::base::State *xBundle) const override;

            /* \brief Project bundle space element onto base space */
            virtual void project(const ompl::base::State *xBundle, ompl::base::State *xBase) const = 0;

            /** @name Fiber space specific operations
             *  @{ */

            /* \brief Lift base space element using a fiber bundle element */
            virtual void lift(const ompl::base::State *xBase, const ompl::base::State *xFiber,
                              ompl::base::State *xBundle) const = 0;

            /* \brief Project bundle space onto fiber space */
            virtual void projectFiber(const ompl::base::State *xBundle, ompl::base::State *xFiber) const = 0;
            /* \brief Get explicit fiber space representation */
            ompl::base::StateSpacePtr getFiberSpace() const;

            /* \brief Get explicit fiber space sampler */
            ompl::base::StateSamplerPtr getFiberSamplerPtr() const;

            /// Dimension of Fiber Space
            virtual unsigned int getFiberDimension() const;
            virtual std::string getFiberTypeAsString() const;

            bool isFibered() const override;

            /* \brief Create explicit fiber space representation */
            virtual void makeFiberSpace();

            /** @} */

        protected:
            virtual ompl::base::StateSpacePtr computeFiberSpace() = 0;

            base::StateSpacePtr fiberSpace_{nullptr};

            base::SpaceInformationPtr siFiberSpace_{nullptr};

            base::StateSamplerPtr fiberSpaceSampler_;

            // \brief A temporary state on Fiber space
            ompl::base::State *xFiberTmp_{nullptr};
        };

        /* \brief A compound projection where each projection is fibered, so
         * that we can create a joint fiber space which can be explicitly
         * accessed by casting this class to a FiberedProjection. */
        class CompoundFiberedProjection : public FiberedProjection, public CompoundProjection
        {
        public:
            CompoundFiberedProjection(base::StateSpacePtr bundleSpace, base::StateSpacePtr baseSpace) = delete;
            CompoundFiberedProjection(const base::StateSpacePtr &bundleSpace, const base::StateSpacePtr &baseSpace,
                               const std::vector<ProjectionPtr> &components);

            ~CompoundFiberedProjection() override = default;

            /** @name Override methods from CompoundProjection
                @{ */
            void lift(const ompl::base::State *xBase, ompl::base::State *xBundle) const override;
            void project(const ompl::base::State *xBundle, ompl::base::State *xBase) const override;

            /// Dimension of Base Space
            unsigned int getBaseDimension() const override;
            /// Dimension of Bundle Space
            unsigned int getDimension() const override;
            /// Dimension of Bundle - Dimension of Base
            unsigned int getCoDimension() const override;

            bool isFibered() const override;

            bool isCompound() const override;

            void print(std::ostream &out) const override;

            /** @} */

            /** @name Override methods from FiberedProjection
                @{ */

            /* \brief Lift base space element using a fiber bundle element */
            void lift(const ompl::base::State *xBase, const ompl::base::State *xFiber,
                              ompl::base::State *xBundle) const override;

            /* \brief Project bundle space onto fiber space */
            void projectFiber(const ompl::base::State *xBundle, ompl::base::State *xFiber) const override;

            /** @} */
        protected:
            ompl::base::StateSpacePtr computeFiberSpace() override;
        };
    }
}

#endif
