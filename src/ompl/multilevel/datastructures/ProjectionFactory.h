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
#include <ompl/multilevel/datastructures/ProjectionTypes.h>
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>

namespace ompl
{
    namespace multilevel
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::multilevel::Projection */
        OMPL_CLASS_FORWARD(Projection);
        /// @endcond
    }
    namespace multilevel
    {
        /* \brief If no projection operator is provided, you can invoke this
         * projection component factory, which tries to guess the projection
         * mapping. For example, if you specify SE3 and R3 as bundle and base,
         * this factory would return the projection onto the R3 subspace in SE3. */
        class ProjectionFactory
        {
        public:
            ProjectionFactory() = default;

            /** \brief Guess projection(s) between two SpaceInformationPtr Bundle and Base */
            // std::vector<ProjectionPtr> MakeProjections(const base::SpaceInformationPtr &Bundle,
            //                                            const base::SpaceInformationPtr &Base);

            // std::vector<ProjectionPtr> MakeProjections(const base::SpaceInformationPtr &Bundle);

            ProjectionPtr makeProjection(const base::SpaceInformationPtr &Bundle,
                                         const base::SpaceInformationPtr &Base);

            ProjectionPtr makeProjection(const base::SpaceInformationPtr &Bundle);

            // ProjectionPtr MakeProjections(const base::SpaceInformationPtr &Bundle);

        protected:
            ProjectionPtr makeProjection(const base::StateSpacePtr &BundleSpace, const base::StateSpacePtr &BaseSpace,
                                         bool areValidityCheckersEquivalent);

            ProjectionPtr makeProjection(const base::StateSpacePtr &BundleSpace);

            /** \brief Guess the projection type from the list of projections in
             * ompl::multilevel::ProjectionTypes */
            ProjectionType identifyProjectionType(const base::StateSpacePtr &BundleSpace,
                                                  const base::StateSpacePtr &BaseSpace);

            /** \brief Check if the mapping is an identity mapping */
            bool isMapping_Identity(const base::StateSpacePtr &, const base::StateSpacePtr &);
            /** \brief Check if the mapping is an empty projection */
            bool isMapping_EmptyProjection(const base::StateSpacePtr &, const base::StateSpacePtr &);
            /** \brief Check if mapping is
             * \f$ \mathbb{R}^N \f$ to \f$ \mathbb{R}^M \f$ */
            bool isMapping_RN_to_RM(const base::StateSpacePtr &, const base::StateSpacePtr &);
            /** \brief Check if mapping is
             * \f$ \mathrm{SE}(2) \f$ to \f$ \mathbb{R}^2 \f$ */
            bool isMapping_SE2_to_R2(const base::StateSpacePtr &, const base::StateSpacePtr &);
            /** \brief Check if mapping is
             * \f$ \mathrm{SE}(2)\times \mathbb{R}^N \f$ to \f$ \mathbb{R}^2 \f$ */
            bool isMapping_SE2RN_to_R2(const base::StateSpacePtr &, const base::StateSpacePtr &);
            /** \brief Check if mapping is
             * \f$ \mathrm{SE}(2)\times \mathbb{R}^N \f$ to \f$ \mathrm{SE}(2) \f$ */
            bool isMapping_SE2RN_to_SE2(const base::StateSpacePtr &, const base::StateSpacePtr &);
            /** \brief Check if mapping is
             * \f$ \mathrm{SE}(2)\times \mathbb{R}^N \f$ to \f$ \mathrm{SE}(2)\times \mathbb{R}^M \f$ */
            bool isMapping_SE2RN_to_SE2RM(const base::StateSpacePtr &, const base::StateSpacePtr &);
            /** \brief Check if mapping is
             * \f$ \mathrm{SE}(3) \f$ to \f$ \mathbb{R}^3 \f$ */
            bool isMapping_SE3_to_R3(const base::StateSpacePtr &, const base::StateSpacePtr &);
            /** \brief Check if mapping is
             * \f$ \mathrm{SE}(3)\times \mathbb{R}^N \f$ to \f$ \mathbb{R}^3 \f$ */
            bool isMapping_SE3RN_to_R3(const base::StateSpacePtr &, const base::StateSpacePtr &);
            /** \brief Check if mapping is
             * \f$ \mathrm{SE}(3)\times \mathbb{R}^N \f$ to \f$ \mathrm{SE}(3) \f$ */
            bool isMapping_SE3RN_to_SE3(const base::StateSpacePtr &, const base::StateSpacePtr &);
            /** \brief Check if mapping is
             * \f$ \mathrm{SE}(3)\times \mathbb{R}^N \f$ to \f$ \mathrm{SE}(3)\times \mathbb{R}^M \f$ */
            bool isMapping_SE3RN_to_SE3RM(const base::StateSpacePtr &, const base::StateSpacePtr &);
            /** \brief Check if mapping is
             * \f$ \mathrm{SO}(2)\times \mathbb{R}^N \f$ to \f$ \mathrm{SO}(2) \f$ */
            bool isMapping_SO2RN_to_SO2(const base::StateSpacePtr &, const base::StateSpacePtr &);
            /** \brief Check if mapping is
             * \f$ \mathrm{SO}(2)\times \mathbb{R}^N \f$ to \f$ \mathrm{SO}(2)\times \mathbb{R}^M \f$ */
            bool isMapping_SO2RN_to_SO2RM(const base::StateSpacePtr &, const base::StateSpacePtr &);
            /** \brief Check if mapping is
             * \f$ \mathrm{SO}^N(2) \f$ to \f$ \mathrm{SO}^M(2) \f$ */
            bool isMapping_SO2N_to_SO2M(const base::StateSpacePtr &, const base::StateSpacePtr &);
            /** \brief Check if mapping is
             * \f$ \mathrm{SO}(3)\times \mathbb{R}^N \f$ to \f$ \mathrm{SO}(3) \f$ */
            bool isMapping_SO3RN_to_SO3(const base::StateSpacePtr &, const base::StateSpacePtr &);
            /** \brief Check if mapping is
             * \f$ \mathrm{SO}(3)\times \mathbb{R}^N \f$ to \f$ \mathrm{SO}(3)\times \mathbb{R}^M \f$ */
            bool isMapping_SO3RN_to_SO3RM(const base::StateSpacePtr &, const base::StateSpacePtr &);
            /** \brief Check if mapping is
             * \f$ \mathbb{R}^N \times \mathrm{SO}(2) \f$ to \f$ \mathbb{R}^N \f$ */
            bool isMapping_RNSO2_to_RN(const base::StateSpacePtr &, const base::StateSpacePtr &);

            /** \brief Check if mapping is
             * \f$ X\times \mathbb{R}^N \f$ to \f$ X \times \mathbb{R}^M \f$
             * whereby \f$ X = \{\mathrm{SO}(2),\mathrm{SO}(3),\mathrm{SE}(2),\mathrm{SE}(3)\} \f$ */
            bool isMapping_XRN_to_XRM(const base::StateSpacePtr &, const base::StateSpacePtr &,
                                      const base::StateSpaceType);

            /** \brief Check if mapping is
             * \f$ X\times \mathbb{R}^N \f$ to \f$ X \f$
             * whereby \f$ X = \{\mathrm{SO}(2),\mathrm{SO}(3),\mathrm{SE}(2),\mathrm{SE}(3)\} \f$ */
            bool isMapping_XRN_to_X(const base::StateSpacePtr &, const base::StateSpacePtr &,
                                    const base::StateSpaceType);

            /** \brief Estimate number of components on state space */
            int GetNumberOfComponents(const base::StateSpacePtr &space);
        };
    }
}
#endif
