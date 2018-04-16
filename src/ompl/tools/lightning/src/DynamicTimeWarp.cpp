/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK, The University of Tokyo.
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
 *   * Neither the name of the JSK, The University of Tokyo nor the names of its
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

/* Author: Dave Coleman
 */

#include <ompl/tools/lightning/DynamicTimeWarp.h>

#include <utility>

namespace  // anonymous
{
    /**
     * \brief Calculate min for 3 numbers
     */
    inline double min3(double n1, double n2, double n3)
    {
        return std::min(n1, std::min(n2, n3));
    }
}  // namespace

ompl::tools::DynamicTimeWarp::DynamicTimeWarp(base::SpaceInformationPtr si) : si_(std::move(si)), table_(1, 1)
{
    table_(0, 0) = 0.;
}

double ompl::tools::DynamicTimeWarp::calcDTWDistance(const og::PathGeometric &path1,
                                                     const og::PathGeometric &path2) const
{
    // Get lengths
    std::size_t n = path1.getStateCount();
    std::size_t m = path2.getStateCount();
    std::size_t nrows = table_.rows(), ncols = table_.cols();

    // Intialize table
    if (nrows <= n || ncols <= m)
    {
        table_.resize(n + 1, m + 1);
        for (std::size_t i = nrows; i <= n; ++i)
            table_(i, 0) = std::numeric_limits<double>::infinity();
        for (std::size_t i = ncols; i <= m; ++i)
            table_(0, i) = std::numeric_limits<double>::infinity();
    }

    // Do calculations
    double cost;
    for (std::size_t i = 1; i <= n; ++i)
        for (std::size_t j = 1; j <= m; ++j)
        {
            cost = si_->distance(path1.getState(i - 1), path2.getState(j - 1));
            table_(i, j) = cost + min3(table_(i - 1, j), table_(i, j - 1), table_(i - 1, j - 1));
        }

    return table_(n, m);
}

double ompl::tools::DynamicTimeWarp::getPathsScore(const og::PathGeometric &path1, const og::PathGeometric &path2) const
{
    // Copy the path but not the states
    og::PathGeometric newPath1 = path1;
    og::PathGeometric newPath2 = path2;

    // Interpolate both paths so that we have an even discretization of samples
    newPath1.interpolate();
    newPath2.interpolate();

    // compute the DTW between two vectors and divide by total path length of the longer path
    double max_states = std::max(newPath1.getStateCount(), newPath2.getStateCount());

    // Prevent division by zero
    if (max_states == 0)
        return std::numeric_limits<double>::max();  // the worse score possible

    return calcDTWDistance(newPath1, newPath2) / max_states;
}
