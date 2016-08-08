/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Rice University
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

/* Author: Mark Moll */

#ifndef OMPL_DATASTRUCTURES_PERMUTATION_
#define OMPL_DATASTRUCTURES_PERMUTATION_

#include <random>

namespace ompl
{
    /// \brief A permutation of indices into an array
    ///
    /// This class tends to be faster than the two-argument version of
    /// std::random_shuffle when permute is called several times, since
    /// the random number generator doesn't need to be allocated each time.
    class Permutation : public std::vector<int>
    {
    public:
        /// \brief Create a permutation of the numbers 0, ... , n - 1
        Permutation(std::size_t n) : std::vector<int>(n)
        {
            permute(n);
        }
        /// \brief Create a permutation of the numbers 0, ..., n - 1
        void permute(unsigned int n)
        {
            if (size() < n)
                resize(n);
            for (unsigned int i = 0; i < n; ++i)
                operator[](i) = i;
            std::shuffle(begin(), begin() + n, generator_);
        }

    private:
        /// Mersenne twister random number generator
        std::mt19937 generator_;
    };
}

#endif
