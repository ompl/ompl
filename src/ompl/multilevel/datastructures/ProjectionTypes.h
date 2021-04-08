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

#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_PROJECTION_TYPES
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_PROJECTION_TYPES

namespace ompl
{
    namespace multilevel
    {
        enum ProjectionType
        {
            /** \brief ompl::multilevel::Projection_None */
            PROJECTION_NONE = 0,

            /** \brief ompl::multilevel::Projection_EmptySet */
            PROJECTION_EMPTY_SET = 1,

            /** \brief ompl::multilevel::Projection_Identity */
            PROJECTION_IDENTITY = 2,

            /** \brief ompl::multilevel::Projection_Relaxation */
            PROJECTION_CONSTRAINED_RELAXATION = 3,

            /** \brief RN \rightarrow RM, m < n */
            PROJECTION_RN_RM = 4,

            /** \brief SE2 \rightarrow R2 */
            PROJECTION_SE2_R2 = 5,

            /** \brief SE2RN \rightarrow R2 */
            PROJECTION_SE2RN_R2 = 6,

            /** \brief SE2RN \rightarrow SE2 */
            PROJECTION_SE2RN_SE2 = 7,

            /** \brief SE2RN \rightarrow SE2RM, m < n */
            PROJECTION_SE2RN_SE2RM = 8,

            /** \brief SO2RN \rightarrow SO2 */
            PROJECTION_SO2RN_SO2 = 9,

            /** \brief SO2RN \rightarrow SO2RM, m < n */
            PROJECTION_SO2RN_SO2RM = 10,

            /** \brief SE3 \rightarrow R3 */
            PROJECTION_SE3_R3 = 11,

            /** \brief SE3RN \rightarrow R3 */
            PROJECTION_SE3RN_R3 = 12,

            /** \brief SE3RN \rightarrow SE3 */
            PROJECTION_SE3RN_SE3 = 13,

            /** \brief SE3RN \rightarrow SE3RM, m < n */
            PROJECTION_SE3RN_SE3RM = 14,

            /** \brief SO3RN \rightarrow SO3 */
            PROJECTION_SO3RN_SO3 = 15,

            /** \brief SO3RN \rightarrow SO3RM, m < n */
            PROJECTION_SO3RN_SO3RM = 16,

            /** \brief RNSO2 \rightarrow RN, n > 0 */
            PROJECTION_RNSO2_RN = 17,

            /** \brief SO2N \rightarrow SO2M (N copies of SO2 onto M copies of SO2) */
            PROJECTION_SO2N_SO2M = 18,

            /** \brief X \rightarrow T (A mapping from state space X to a task
             * space T) */
            PROJECTION_TASK_SPACE = 19,

            /** \brief Compound projection */
            PROJECTION_COMPOUND = -2,

            /** \brief Unknown projection */
            PROJECTION_UNKNOWN = -1
        };
    }
}

#endif
