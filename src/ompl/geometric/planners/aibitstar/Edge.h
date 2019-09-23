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

// Authors: Marlin Strub

#ifndef OMPL_GEOMETRIC_PLANNERS_AIBITSTAR_EDGE_
#define OMPL_GEOMETRIC_PLANNERS_AIBITSTAR_EDGE_

#include <array>
#include <limits>
#include <memory>

#include "ompl/base/Cost.h"

namespace ompl
{
    namespace geometric
    {
        namespace aibitstar
        {
            // Forward declaration of vertex class.
            class Vertex;

            struct Edge
            {
                /** \brief OMPL's heap unfortunately only works for default constructable element. */
                Edge() = default;

                /** \brief Construct the edge by providing values for all member variables. */
                Edge(const std::shared_ptr<Vertex> &parent, const std::shared_ptr<Vertex> &child,
                     const ompl::base::Cost &heuristicCost, const std::array<double, 3u> &key);

                /** \brief Destruct the edge. */
                ~Edge() = default;

                /** \brief The parent vertex of this edge. */
                std::shared_ptr<Vertex> parent;

                /** \brief The child vertex of this edge. */
                std::shared_ptr<Vertex> child;

                /** \brief The heuristic cost of this edge. */
                ompl::base::Cost heuristicCost{std::numeric_limits<double>::signaling_NaN()};

                /** \brief The sort key of this edge. */
                std::array<double, 3u> key{std::numeric_limits<double>::signaling_NaN(),
                                           std::numeric_limits<double>::signaling_NaN(),
                                           std::numeric_limits<double>::signaling_NaN()};
            };

        }  // namespace aibitstar

    }  // namespace geometric

}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_AIBITSTAR_EDGE_
