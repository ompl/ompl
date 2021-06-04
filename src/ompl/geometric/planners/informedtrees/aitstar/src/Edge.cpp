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

// Authors: Marlin Strub

#include "ompl/geometric/planners/informedtrees/aitstar/Edge.h"

namespace ompl
{
    namespace geometric
    {
        namespace aitstar
        {
            Edge::Edge()
              : parent_()
              , child_()
              , sortKey_({ompl::base::Cost(std::numeric_limits<double>::signaling_NaN()),
                          ompl::base::Cost(std::numeric_limits<double>::signaling_NaN()),
                          ompl::base::Cost(std::numeric_limits<double>::signaling_NaN())})
            {
            }

            Edge::Edge(const std::shared_ptr<Vertex> &parent, const std::shared_ptr<Vertex> &child,
                       const std::array<ompl::base::Cost, 3u> &sortKey)
              : parent_(parent), child_(child), sortKey_(sortKey)
            {
            }

            std::shared_ptr<Vertex> Edge::getParent() const
            {
                return parent_;
            }

            std::shared_ptr<Vertex> Edge::getChild() const
            {
                return child_;
            }

            const std::array<ompl::base::Cost, 3u> &Edge::getSortKey() const
            {
                return sortKey_;
            }

            void Edge::setSortKey(const std::array<ompl::base::Cost, 3u> &key)
            {
                sortKey_ = key;
            }

        }  // namespace aitstar

    }  // namespace geometric

}  // namespace ompl
