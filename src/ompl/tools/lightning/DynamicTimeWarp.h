/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK, The University of Tokyo.
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
 *   * Neither the name of the JSK, The University of Tokyo nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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

/* Author: Dave Coleman
   Desc:   Compute Dynamic Time Warping
*/

#ifndef OMPL_TOOLS_LIGHTNING_DYNAMIC_TIME_WARP_
#define OMPL_TOOLS_LIGHTNING_DYNAMIC_TIME_WARP_

#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/SpaceInformation.h>

#include <Eigen/Core>

namespace ompl
{
    namespace tools
    {
        namespace og = ompl::geometric;

        /// @cond IGNORE
        OMPL_CLASS_FORWARD(DynamicTimeWarp);
        /// @endcond

        /** \class ompl::geometric::DynamicTimeWarpPtr
            \brief A shared pointer wrapper for ompl::tools::DynamicTimeWarp */

        class DynamicTimeWarp
        {
        public:
            explicit DynamicTimeWarp(base::SpaceInformationPtr si);

            /**
             * \brief Use Dynamic Timewarping to score two paths
             * \param path1
             * \param path2
             * \return score
             */
            double calcDTWDistance(const og::PathGeometric &path1, const og::PathGeometric &path2) const;

            /**
             * \brief Use dynamic time warping to compare the similarity of two paths
             *        Note: this will interpolate both of the paths and it returns the change by reference
             *        Note: before calling this function you might want to reverse one of the paths so that their
             *        start and goals are property aligned (and match better)
             * \param path1
             * \param path2
             * \return score
             */
            double getPathsScore(const og::PathGeometric &path1, const og::PathGeometric &path2) const;

        private:
            /** \brief The created space information */
            base::SpaceInformationPtr si_;

            /** \brief Distance matrix */
            mutable Eigen::MatrixXd table_;

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };  // end of class

    }  // namespace tools
}  // namespace ompl

#endif
