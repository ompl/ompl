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

#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_PATH_RESTRICTION_FIND_SECTION_SIDESTEP_
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_PATH_RESTRICTION_FIND_SECTION_SIDESTEP_
#include <ompl/multilevel/datastructures/pathrestriction/FindSection.h>

namespace ompl
{
    namespace multilevel
    {
        /**
           @anchor FindSectionSideStep

           @par Short description
           FindSectionSideStep tries to solve the find section problem by
           going along an L1 interpolation towards the goal while no collision
           occurs. Once a collision occurs, we sample randomly on the current
           fiber and make side steps towards feasible samples. We then
           recursively continue from those samples.

           @par External documentation
           Andreas Orthey and Sohaib Akbar and Marc Toussaint,
           Multilevel Motion Planning: A Fiber Bundle Formulation,
           in <em>arXiv:2007.09435 [cs.RO]</em>, 2020,
           [[PDF]](https://arxiv.org/pdf/2007.09435.pdf)
       */

        class FindSectionSideStep : public FindSection
        {
            using BaseT = FindSection;

        public:
            FindSectionSideStep() = delete;
            FindSectionSideStep(PathRestriction *);

            virtual ~FindSectionSideStep();

            virtual bool solve(HeadPtr &head) override;

            bool recursiveSideStep(HeadPtr &head, bool interpolateFiberFirst = true, unsigned int depth = 0);
        };
    }
}

#endif
