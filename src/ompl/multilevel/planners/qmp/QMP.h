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

/* Author: Andreas Orthey, Sohaib Akbar */

#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_QMP_
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_QMP_
#include <ompl/multilevel/datastructures/BundleSpaceSequence.h>
#include <ompl/multilevel/planners/qmp/QMPImpl.h>

namespace ompl
{
    namespace multilevel
    {
        /**
             @anchor QMP

             @par Short description
             The Quotient-space roadMap Planner (QMP) is
             a multi-query, probabilistically complete roadmap planner which
             generalizes PRM to bundle spaces.

             @par External documentation
             A. Orthey, A. Escande and E. Yoshida,
             Quotient-Space Motion Planning,
             in <em>International Conference on Intelligent Robots and Systems</em>, 2018,
             [[PDF]](https://arxiv.org/abs/1807.09468)
        */

        /** \brief [Q]uotient space road[M]ap [P]lanner (QMP) Algorithm */
        using QMP = BundleSpaceSequence<QMPImpl>;

    }  // namespace multilevel
}  // namespace ompl

#endif
