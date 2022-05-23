/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, University of Oxford
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
 *   * Neither the name of the University of Toronto nor the names of its
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

// Authors: Valentin Hartmann

#ifndef OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_EIRMSTAR_
#define OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_EIRMSTAR_

#include "ompl/geometric/planners/informedtrees/EITstar.h"

namespace ompl
{
    namespace geometric
    {
        /**
        @anchor gEIRMstar

        \ref gEIRMstar "EIRM*" (Effort Informed Roadmaps) is an almost-surely asymptotically optimal multiquery path
        planner based on EIT*. It aims to find an initial solution quickly by resuing previously invested computational
        effort (i.e., validated edges) and asymptotically converges to the globally optimal solution.

        It uses an asymmetric bidirectional search to identify known valid edges in a lazy roadmap and then reuses these
        to quickly solve individual planning queries by reducing computational effort. As EIT*, EIRM* then adds batches
        of samples to its approximation when improving the solution. To avoid issues that come with the growing graph
        size from refining the approximation to find the globally optimal solution, EIRM* resets its approximation to
        the initial batch of samples at each new query in a multiquery problem.

        @par Associated publications:

        V. N. Hartmann, M. P. Strub, M. Toussaint, J. D. Gammell. "Effort informed roadmaps (EIRM*):
        Efficient asymptotically optimal multiquery planning by actively reusing validation effort"
        Submitted to Proceedings of the International Symposium on Robotics Research (ISRR) 2022

        arXiv: <a href="https://arxiv.org/abs/2205.08480">arXiv:2205.08480</a>
        Video 1: <a href="https://www.youtube.com/watch?v=OjcnjuJLVUY">ISRR trailer</a>

        */
        /** \brief Effort Informed Roadmaps (EIRM*) */
        class EIRMstar : public EITstar
        {
        public:
            /** \brief Constructs an instance of EIRM* using the provided space information. */
            explicit EIRMstar(const std::shared_ptr<ompl::base::SpaceInformation> &spaceInfo);

            /** \brief Set start/goal pruning threshold. */
            void setStartGoalPruningThreshold(unsigned int threshold);

            /** \brief Get threshold at which we prune starts/goals. */
            unsigned int getStartGoalPruningThreshold() const;
        };

    }  // namespace geometric

}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_EIRMSTAR_
