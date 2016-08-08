/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Rice University
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

/* Author: Matt Maly */

#ifndef OMPL_CONTROL_PLANNERS_LTL_PROPOSITIONALDECOMPOSITION_
#define OMPL_CONTROL_PLANNERS_LTL_PROPOSITIONALDECOMPOSITION_

#include "ompl/base/State.h"
#include "ompl/control/planners/syclop/Decomposition.h"
#include "ompl/control/planners/ltl/World.h"
#include "ompl/util/ClassForward.h"
#include "ompl/util/RandomNumbers.h"
#include "ompl/base/StateSampler.h"
#include <vector>

namespace ompl
{
    namespace control
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::control::PropositionalDecomposition */
        OMPL_CLASS_FORWARD(PropositionalDecomposition);
        /// @endcond

        /** \class ompl::control::PropositionalDecompositionPtr
            \brief A shared pointer wrapper for ompl::control::PropositionalDecomposition */

        /** \brief A propositional decomposition wraps a given Decomposition
            with a region-to-proposition assignment operator.
            Each region in the decomposition has a corresponding World. */
        class PropositionalDecomposition : public Decomposition
        {
        public:
            /** \brief Creates a propositional decomposition wrapped around a given decomposition
                with a given number of propositions. */
            PropositionalDecomposition(const DecompositionPtr &decomp);

            /** \brief Clears all memory belonging to this propositional decomposition. */
            ~PropositionalDecomposition() override;

            /** \brief Returns the World corresponding to a given region. */
            virtual World worldAtRegion(int rid) = 0;

            /* \todo section off the below methods with a general description of
                "wrapper methods that simply call the underlying decomposition */

            /** \brief Returns the number of regions in this propositional decomposition's
                underlying decomposition. */
            int getNumRegions() const override;

            /** \brief Returns the number of propositions in this propositional decomposition. */
            virtual int getNumProps() const = 0;

            /** \brief Returns the volume of a given region. */
            double getRegionVolume(int rid) override;

            /** \brief Returns the region of the underlying decomposition that contains
                a given State. */
            int locateRegion(const base::State *s) const override;

            void project(const base::State *s, std::vector<double> &coord) const override;

            void getNeighbors(int rid, std::vector<int> &neighbors) const override;

            void sampleFromRegion(int rid, RNG &rng, std::vector<double> &coord) const override;

            void sampleFullState(const base::StateSamplerPtr &sampler, const std::vector<double> &coord,
                                 base::State *s) const override;

        protected:
            DecompositionPtr decomp_;
        };
    }
}
#endif
