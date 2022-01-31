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

#ifndef OMPL_BASE_SPACES_EMPTY_STATE_SPACE_
#define OMPL_BASE_SPACES_EMPTY_STATE_SPACE_
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace ompl
{
    namespace base
    {
        /** \brief A state space representing an empty state space. This is
         * useful for multilevel representations, where some projections might
         * project onto an empty state space. */
        class EmptyStateSpace : public RealVectorStateSpace
        {
        public:
            EmptyStateSpace()
              : RealVectorStateSpace(0)
            {
                setName("EmptySpace");
            }
            ~EmptyStateSpace() override = default;

            double getMeasure() const override
            {
              return 0;
            }

            void setup() override
            {
              return;
            }

            unsigned int getDimension() const override
            {
              return 0;
            }

            double getMaximumExtent() const override
            {
              return 0;
            }

        };
    }
}

#endif
