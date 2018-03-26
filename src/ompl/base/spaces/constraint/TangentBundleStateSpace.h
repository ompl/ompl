/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Rice University
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
*   * Neither the name of the Rice University nor the names of its
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

/* Author: Zachary Kingston */

#ifndef OMPL_BASE_SPACES_TANGENTBUNDLE_STATE_SPACE_
#define OMPL_BASE_SPACES_TANGENTBUNDLE_STATE_SPACE_

#include "ompl/base/spaces/constraint/AtlasStateSpace.h"

namespace ompl
{
    namespace base
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::TangentBundleStateSpace */
        OMPL_CLASS_FORWARD(TangentBundleStateSpace);
        /// @endcond

        /**
           @anchor gTangentBundle
           @par Short Description
           TangentBundleStateSpace implements a lazy atlas-based methodology for constrained sampling-based planning,
           where the underlying constraint manifold is locally parameterized by \e charts (AtlasChart). The underlying
           constraint manifold can then be sampled and explored using the collection of these charts (an \e atlas). The
           difference between TangentBundleStateSpace and AtlasStateSpace is three-fold: TangentBundleStateSpace takes a
           lazy approach to evaluating the constraint function when traversing the manifold, TangentBundleStateSpace
           uses biased sampling for selected charts by default, and TangentBundleStateSpace does not use halfspace
           separation of charts.

           @par External Documentation
           This state space is inspired by the work on Tangent Bundle RRT.

           B. Kim, T. T. Um, C. Suh, and F. C. Park, "Tangent bundle RRT: A randomized algorithm for constrained motion
           planning," Robotica 34.1 (2016): 202-225. DOI:
           <a href="http://dx.doi.org/10.1017/S0263574714001234">10.1017/S0263574714001234</a>.

           For more information on constrained sampling-based planning using atlas-based methods, see the following
           review paper. The section on atlas-based methods cites most of the relevant literature.

           Z. Kingston, M. Moll, and L. E. Kavraki, “Sampling-Based Methods for Motion Planning with Constraints,”
           Annual Review of Control, Robotics, and Autonomous Systems, 2018. DOI:
           <a href="http://dx.doi.org/10.1146/annurev-control-060117-105226">10.1146/annurev-control-060117-105226</a>
           <a href="http://kavrakilab.org/publications/kingston2018sampling-based-methods-for-motion-planning.pdf">[PDF]</a>.
        */

        /** \brief ConstrainedStateSpace encapsulating a planner-agnostic lazy
         * atlas algorithm for planning on a constraint manifold. */
        class TangentBundleStateSpace : public AtlasStateSpace
        {
        public:
            TangentBundleStateSpace(const StateSpacePtr &ambientSpace, const ConstraintPtr &constraint);

            /** \brief Do sanity checks, minus geodesic constraint
             * satisfiability (as this is a lazy method). */
            void sanityChecks() const override;

            /** \brief Traverse the manifold from \a from toward \a to. Returns
             * true if we reached \a to, and false if we stopped early for any
             * reason, such as a collision or traveling too far. No collision
             * checking is performed if \a interpolate is true. If \a geodesic
             * is not nullptr, the sequence of intermediates is saved to it,
             * including a copy of \a from, as well as the final state, which is
             * a copy of \a to if we reached \a to. Caller is responsible for
             * freeing states returned in \a geodesic.*/
            bool discreteGeodesic(const State *from, const State *to, bool interpolate = false,
                                  std::vector<State *> *geodesic = nullptr) const override;

            /** \brief Like interpolate(...), but interpolates between
             * intermediate states already supplied in \a stateList from a
             * previous call to discreteGeodesic(..., \a geodesic). The \a from
             * and \a to states are the first and last elements \a stateList.
             * Returns a pointer to a state in \a geodesic. As
             * TangentBundleStateSpace employs a lazy approach to manifold
             * traversal, additional fix-up is required to generate a state that
             * satisfies constraints. */
            State *geodesicInterpolate(const std::vector<State *> &geodesic, double t) const override;

            /** \brief Reproject a state onto the surface of the manifold. Returns true if projection was successful,
             * and the new state is valid. */
            bool project(State *state) const;
        };
    }
}

#endif
