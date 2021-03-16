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

#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_BUNDLE_SUBSPACE_FACTORY__
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_BUNDLE_SUBSPACE_FACTORY__
#include "ProjectionComponent.h"
#include "ProjectionComponentTypes.h"
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>

namespace ompl
{
    namespace multilevel
    {
        OMPL_CLASS_FORWARD(ProjectionComponent);
    }
    namespace multilevel
    {
        /* \brief If no projection operator is provided, you can invoke this
         * projection component factory, which tries to guess the projection
         * mapping. For example, if you specify SE3 and R3 as bundle and base,
         * this factory would return the projection onto the R3 subspace in SE3. */
        class ProjectionComponentFactory
        {
        public:
            ProjectionComponentFactory() = default;

            std::vector<ProjectionComponentPtr> MakeProjectionComponents(
                base::SpaceInformationPtr Bundle,
                base::SpaceInformationPtr Base);

            std::vector<ProjectionComponentPtr> MakeProjectionComponents(
                base::SpaceInformationPtr Bundle);

        protected:
            ProjectionComponentPtr MakeProjectionComponent(
                base::StateSpacePtr BundleSpace,
                base::StateSpacePtr BaseSpace, bool);

            ProjectionComponentPtr MakeProjectionComponent(
                base::StateSpacePtr BundleSpace);

            ProjectionComponentType identifyProjectionComponentType(
                const base::StateSpacePtr BundleSpace,
                const base::StateSpacePtr BaseSpace);

            bool isMapping_Identity(const base::StateSpacePtr, const base::StateSpacePtr);
            bool isMapping_EmptyProjection(const base::StateSpacePtr, const base::StateSpacePtr);
            bool isMapping_RN_to_RM(const base::StateSpacePtr, const base::StateSpacePtr);
            bool isMapping_SE2_to_R2(const base::StateSpacePtr, const base::StateSpacePtr);
            bool isMapping_SE2RN_to_R2(const base::StateSpacePtr, const base::StateSpacePtr);
            bool isMapping_SE2RN_to_SE2(const base::StateSpacePtr, const base::StateSpacePtr);
            bool isMapping_SE2RN_to_SE2RM(const base::StateSpacePtr, const base::StateSpacePtr);
            bool isMapping_SE3_to_R3(const base::StateSpacePtr, const base::StateSpacePtr);
            bool isMapping_SE3RN_to_R3(const base::StateSpacePtr, const base::StateSpacePtr);
            bool isMapping_SE3RN_to_SE3(const base::StateSpacePtr, const base::StateSpacePtr);
            bool isMapping_SE3RN_to_SE3RM(const base::StateSpacePtr, const base::StateSpacePtr);
            bool isMapping_SO2RN_to_SO2(const base::StateSpacePtr, const base::StateSpacePtr);
            bool isMapping_SO2RN_to_SO2RM(const base::StateSpacePtr, const base::StateSpacePtr);
            bool isMapping_SO2N_to_SO2M(const base::StateSpacePtr, const base::StateSpacePtr);
            bool isMapping_SO3RN_to_SO3(const base::StateSpacePtr, const base::StateSpacePtr);
            bool isMapping_SO3RN_to_SO3RM(const base::StateSpacePtr, const base::StateSpacePtr);
            bool isMapping_RNSO2_to_RN(const base::StateSpacePtr, const base::StateSpacePtr);

            bool isMapping_XRN_to_XRM(const base::StateSpacePtr, const base::StateSpacePtr, const base::StateSpaceType);
            bool isMapping_XRN_to_X(const base::StateSpacePtr, const base::StateSpacePtr, const base::StateSpaceType);

            int GetNumberOfComponents(base::StateSpacePtr space);
        };
    }
}
#endif
