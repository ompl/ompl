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

/* Author: Ioan Sucan */

#ifndef OMPL_BASE_SPACES_REAL_VECTOR_BOUNDS_
#define OMPL_BASE_SPACES_REAL_VECTOR_BOUNDS_

#include <vector>

namespace ompl
{
    namespace base
    {
        /** \brief The lower and upper bounds for an R<sup>n</sup> space */
        class RealVectorBounds
        {
        public:
            /** \brief Constructor; \e dim represents the dimension of the space these bounds are for. */
            RealVectorBounds(unsigned int dim)
            {
                resize(dim);
            }

            /** \brief Set the lower bound in each dimension to a specific value */
            void setLow(double value);

            /** \brief Set the upper bound in each dimension to a specific value */
            void setHigh(double value);

            /** \brief Set the lower bound of a dimension to a specific value */
            void setLow(unsigned int index, double value);

            /** \brief Set the upper bound of a dimension to a specific value */
            void setHigh(unsigned int index, double value);

            /** \brief Change the number of dimensions for the bounds */
            void resize(std::size_t size);

            /** \brief Compute the volume of the space enclosed by the bounds */
            double getVolume() const;

            /** \brief Get the difference between the high and low bounds for each dimension: result[i] = high[i] -
             * low[i] */
            std::vector<double> getDifference() const;

            /** \brief Check if the bounds are valid (same length for
                low and high, high[i] > low[i]). Throw an exception if
                this is not the case. */
            void check() const;

            /** \brief Lower bound */
            std::vector<double> low;

            /** \brief Upper bound */
            std::vector<double> high;
        };
    }
}
#endif
