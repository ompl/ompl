/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, University of New Hampshire
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
 *   * Neither the names of the copyright holders nor the names of its
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
/*********************************************************************
 * Attribution Notice:
 *
 * This file contains code partially derived from the AIT* or BIT* planner
 * in the Open Motion Planning Library (OMPL). Components such as priority
 * queue type.
 *
 * Additional modifications for BLIT* were independently
 * developed by Yi Wang.
 *********************************************************************/

// Authors: Yi Wang, Eyal Weiss, Bingxian Mu, Oren Salzman

#ifndef OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_BLITSTAR_QUEUETYPES_
#define OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_BLITSTAR_QUEUETYPES_

#include <array>
#include <functional>
#include <memory>
#include <utility>
#include "ompl/base/Cost.h"
#include "ompl/datastructures/BinaryHeap.h"

namespace ompl
{
    namespace geometric
    {
        namespace blitstar
        {
            // Declarations of BLIT* vertex classes.
            class Vertex;

            /** \brief A type for elements in the vertex queue. */
            using KeyVertexPair = std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>>;
            
            /** \brief A meeting vertex. */
            using MiddleVertex = std::pair<ompl::base::Cost, std::shared_ptr<Vertex>>;
            
            /** \brief A promissing edge. */
            using keyEdgePair  =  std::pair<std::shared_ptr<Vertex>, std::shared_ptr<Vertex>>;  
                   
            /** \brief The type of the vertex queue. */
            using VertexQueue =
                ompl::BinaryHeap<KeyVertexPair, std::function<bool(const KeyVertexPair &, const KeyVertexPair &)>>;

        }  // namespace blitstar
    }  // namespace geometric
}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_BLITSTAR_QUEUETYPES_
