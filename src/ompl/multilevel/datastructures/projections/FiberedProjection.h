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
        /* \brief A bundle projection with an explicit fiber space representation
         * which can be explicitly sampled to lift states */
        class FiberedProjection : public Projection
        {
        public:
            FiberedProjection(base::StateSpacePtr bundleSpace, base::StateSpacePtr baseSpace);

            virtual ~FiberedProjection() = default;

            virtual void lift(const ompl::base::State *xBase, ompl::base::State *xBundle) const override;

            /* \brief Lift base space element using a fiber bundle element */
            virtual void lift(const ompl::base::State *xBase, const ompl::base::State *xFiber,
                              ompl::base::State *xBundle) const = 0;

            /* \brief Project bundle space onto fiber space */
            virtual void projectFiber(const ompl::base::State *xBundle, ompl::base::State *xFiber) const = 0;

            /* \brief Get explicit fiber space representation */
            virtual ompl::base::StateSpacePtr getFiberSpace() const;

            /* \brief Get explicit fiber space sampler */
            ompl::base::StateSamplerPtr getFiberSamplerPtr() const;

            /// Dimension of Fiber Space
            unsigned int getFiberDimension() const;
            std::string getFiberTypeAsString() const;

            bool isFibered() const override;

            /* \brief Create explicit fiber space representation */
            void makeFiberSpace();

        protected:
            virtual ompl::base::StateSpacePtr computeFiberSpace() = 0;

            base::StateSpacePtr fiberSpace_{nullptr};

            base::SpaceInformationPtr siFiberSpace_{nullptr};

            base::StateSamplerPtr fiberSpaceSampler_;

            // \brief A temporary state on Fiber space
            ompl::base::State *xFiberTmp_{nullptr};
        };
    }
}

#endif
