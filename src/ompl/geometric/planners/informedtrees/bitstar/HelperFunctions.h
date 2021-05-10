/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, University of Toronto
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

/* Authors: Jonathan Gammell */

#ifndef OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_BITSTAR_HELPERFUNCTIONS_
#define OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_BITSTAR_HELPERFUNCTIONS_

// std::pair
#include <utility>

////////////////////////////////
// Anonymous helpers
namespace
{
    /** \brief Define the addition operator for a std::pair<T,U> from the addition operators of T and U. */
    template <typename T, typename U>
    std::pair<T, U> operator+(const std::pair<T, U> &lhs, const std::pair<T, U> &rhs)
    {
        return std::make_pair(lhs.first + rhs.first, lhs.second + rhs.second);
    };

    /** \brief Delete an iterator in a vector. */
    template <typename V>
    void swapPopBack(typename V::iterator iter, V *vect)
    {
        // Swap to the back if not already there
        if (iter != (vect->end() - 1))
        {
            std::swap(*iter, vect->back());
        }

        // Delete the back of the vector
        vect->pop_back();
    };

    /** \brief A stream operator for an std::array of Cost */
    template <std::size_t SIZE>
    std::ostream& operator<<(std::ostream& out, const std::array<ompl::base::Cost, SIZE>& costArray)
    {
        // Start with a bracket
        out << "(";

        // Iterate through the values
        for (unsigned int i = 0u; i < costArray.size(); ++i)
        {
            // Print value
            out << costArray.at(i);

            // If not last, print a ,
            if (i != costArray.size() - 1u)
            {
                out << ", ";
            }
        }

        // End with a bracket
        out << ")";

        return out;
    };
}
////////////////////////////////
#endif  // OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_BITSTAR_HELPERFUNCTIONS_
