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

#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_SMLR_
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_SMLR_

#include <ompl/multilevel/datastructures/BundleSpaceSequence.h>
#include <ompl/multilevel/planners/sparse/SMLRImpl.h>

namespace ompl
{
    namespace multilevel
    {
        /**
             @anchor SMLR
             @par Short description
             Sparse Multilevel Roadmap Planner (SMLR) algorithm,
             generalizes the SPARS planner to bundle spaces.

             @par External documentation
             A. Orthey and M. Toussaint,
             Sparse Multilevel Roadmaps on Fiber Bundles for High-Dimensional Motion Planning,
             in arXiv:2011.00832 [cs.RO], 2020
             [[PDF]](https://arxiv.org/pdf/2011.00832.pdf)
        */

        /** \brief [S]parse [M]ulti[L]evel [R]oadmap planner (SMLR) Algorithm */
        using SMLR = BundleSpaceSequence<SMLRImpl>;
    }
}
#endif
