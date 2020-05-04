/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, University of Oxford
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

/* Authors: Marlin Strub */

#ifndef OMPL_GEOMETRIC_PLANNERS_ABITSTAR_ABITSTAR_
#define OMPL_GEOMETRIC_PLANNERS_ABITSTAR_ABITSTAR_

#include "ompl/geometric/planners/bitstar/BITstar.h"

namespace ompl
{
    namespace geometric
    {
        /**
        @anchor gABITstar

        \ref gABITstar "ABIT*" (Advanced Batch Informed Trees) is an almost-surely asymptotically optimal path planner.
        It views the planning problem as the two subproblems of approximation and search. This perspective allows it use
        advanced graph-search techniques, such as inflation and truncation.

        This implementation of ABIT* derives from BIT* which means it benefits from a couple of features not mentioned
        in the ABIT* publication. It can handle multiple starts, multiple goals, a variety of optimization objectives
        (e.g., path length), and with \ref gBITstarSetJustInTimeSampling "just-in-time sampling", infinite problem
        domains. Note that for some of optimization  objectives, the user must specify a suitable heuristic and that
        when this heuristic is not specified, it will use the conservative/always admissible \e zero-heuristic.

        This implementation also includes some new advancements, including the ability to prioritize exploration until
        an initial solution is found (\ref gBITstarSetDelayRewiringUntilInitialSolution "Delayed rewiring"), the ability
        to generate samples only when necessary (\ref gBITstarSetJustInTimeSampling "Just-in-time sampling"), and the
        ability to periodically remove samples that have yet to be connected to the graph (\ref
        gBITstarSetDropSamplesOnPrune "Sample dropping"). With just-in-time sampling, BIT* can even solve planning
        problems with infinite state space boundaries, i.e., (-inf, inf).


        @par Associated publication:

        M. P. Strub, J. D. Gammell. “Advanced BIT* (ABIT*): Sampling-based planning with advanced graph-search
        techniques.” in Proceedings of the IEEE international conference on robotics and automation (ICRA), Paris,
        France, 31 May – 4 Jun. 2020.
        DOI: <a href="https://arxiv.org/abs/2002.06589">arXiv:2002.06589</a>.
        Video: <a href="https://www.youtube.com/watch?v=VFdihv8Lq2A">Illustration video</a>.
        */
        /** \brief Advanced Batch Informed Trees (ABIT*) */
        class ABITstar : public ompl::geometric::BITstar
        {
        public:
            /* \brief Construct with a pointer to the space information and an optional name. */
            ABITstar(const base::SpaceInformationPtr &spaceInfo, const std::string &name = "ABITstar");

            /* \brief Destruct using the default destructor. */
            virtual ~ABITstar() override = default;

            /** \brief Set the inflation factor for the initial search. */
            void setInitialInflationFactor(double factor);

            /** \brief Set the parameter for the inflation factor update policy. */
            void setInflationFactorParameter(double parameter);

            /** \brief Set the parameter for the truncation factor update policy. */
            void setTruncationFactorParameter(double parameter);

            /** \brief Get the inflation factor for the initial search. */
            double getInitialInflationFactor() const;

            /** \brief Get the inflation factor for the current search. */
            double getCurrentInflationFactor() const;

            /** \brief Get the truncation factor for the current search. */
            double getCurrentTruncationFactor() const;
        };

    }  // namespace geometric

}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_ABITSTAR_ABITSTAR_
