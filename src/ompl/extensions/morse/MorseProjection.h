/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
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

/* Author: Caleb Voss */

#ifndef OMPL_EXTENSION_MORSE_PROJECTION_
#define OMPL_EXTENSION_MORSE_PROJECTION_

#include "ompl/extensions/morse/MorseStateSpace.h"

namespace ompl
{
    namespace base
    {
        /** \brief This class implements a generic projection for the MorseStateSpace,
            namely, the subspace representing the x and y positions of every rigid body */
        class MorseProjection : public ProjectionEvaluator
        {
        public:
            /** \brief Construct a projection evaluator for a specific state space */
            MorseProjection(const StateSpacePtr &space);

            /** \brief Perform configuration steps, if needed */
            void setup() override;

            /** \brief Return the dimension of the projection defined by this evaluator */
            unsigned int getDimension() const override;

            /** \brief Set the default cell dimensions for this
                projection. The default implementation of this
                function sets the size to 1.0 for all dimensions.
                setup() calls this function if no cell
                dimensions have been previously set. */
            void defaultCellSizes() override;

            /** \brief Compute the projection as an array of double values */
            void project(const State *state, Eigen::Ref<Eigen::VectorXd> projection) const override;

        protected:
            /** \brief The state space this projection operates on */
            MorseStateSpace *space_;
        };
    }
}

#endif
