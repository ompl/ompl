/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

#ifndef OMPL_GEOMETRIC_PATH_HYBRIDIZATION_
#define OMPL_GEOMETRIC_PATH_HYBRIDIZATION_

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/geometric/PathGeometric.h"
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <iostream>
#include <set>

namespace ompl
{
    namespace geometric
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::geometric::PathHybridization */
        OMPL_CLASS_FORWARD(PathHybridization);
        /// @endcond

        /** \class ompl::geometric::PathHybridizationPtr
            \brief A shared pointer wrapper for ompl::geometric::PathHybridization */

        /** \brief Given multiple geometric paths, attempt to combine them in order to obtain a shorter solution.

            @par External documentation

            B. Raveh, A. Enosh, and D. Halperin,
            A little more, a lot better: Improving path quality by a path-merging algorithm,
            <em>IEEE Trans. on Robotics</em>, vol. 27, pp. 365–371, Apr. 2011.
            DOI: [10.1109/TRO.2010.2098622](http://dx.doi.org/10.1109/TRO.2010.2098622)<br>
            [[PDF]](http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=5686946)
        */
        class PathHybridization
        {
        public:
            /** \brief The constructor needs to know about the space information of the paths it will operate on */
            PathHybridization(base::SpaceInformationPtr si);

            /** \brief This constructor also takes an alternative Optimization Objective to find lower costs for
             *         arbitrary objectives. */
            PathHybridization(base::SpaceInformationPtr si, base::OptimizationObjectivePtr obj);

            ~PathHybridization();

            /** \brief Get the currently computed hybrid path. computeHybridPath() needs to have been called before. */
            const geometric::PathGeometricPtr &getHybridPath() const;

            /** \brief Run Dijkstra's algorithm to find out the lowest-cost path among the mixed ones */
            void computeHybridPath();

            /** \brief Add a path to the hybridization. If \e matchAcrossGaps is true, more possible edge connections
               are evaluated.
                Return the number of attempted connections between paths. */
            unsigned int recordPath(const geometric::PathGeometricPtr &pp, bool matchAcrossGaps);

            /** \brief Get the number of paths that are currently considered as part of the hybridization */
            std::size_t pathCount() const;

            /** \brief Given two geometric paths \e p and \e q, compute the alignment of the paths using dynamic
                programming in an edit-distance like fashion, as described in the referenced paper. The cost of a gap
                is considered to be \e gapCost. The output of the computation is two arrays \e indexP and \e indexQ of
                equal length, such that these arrays contain matching index positions from the states in \e p and \e q,
                respectively. Gaps are marked by -1. */
            void matchPaths(const geometric::PathGeometric &p, const geometric::PathGeometric &q, double gapValue,
                            std::vector<int> &indexP, std::vector<int> &indexQ) const;

            /** \brief Clear all the stored paths */
            void clear();

            /** \brief Print information about the computed path */
            void print(std::ostream &out = std::cout) const;

            /** \brief Get the name of the algorithm */
            const std::string &getName() const;

        private:
            /// @cond IGNORE
            struct vertex_state_t
            {
                using kind = boost::vertex_property_tag;
            };

            using HGraph = boost::adjacency_list<
                boost::vecS, boost::vecS, boost::undirectedS,
                boost::property<vertex_state_t, base::State *,
                                boost::property<boost::vertex_predecessor_t, unsigned long int,
                                                boost::property<boost::vertex_rank_t, base::Cost>>>,
                boost::property<boost::edge_weight_t, base::Cost>>;

            using Vertex = boost::graph_traits<HGraph>::vertex_descriptor;
            using Edge = boost::graph_traits<HGraph>::edge_descriptor;

            struct PathInfo
            {
                PathInfo(const geometric::PathGeometricPtr &path)
                  : path_(path), states_(path->getStates()), cost_(base::Cost())
                {
                    vertices_.reserve(states_.size());
                }

                bool operator==(const PathInfo &other) const
                {
                    return path_ == other.path_;
                }

                bool operator<(const PathInfo &other) const
                {
                    return path_ < other.path_;
                }

                geometric::PathGeometricPtr path_;
                const std::vector<base::State *> &states_;
                base::Cost cost_;
                std::vector<Vertex> vertices_;
            };
            /// @endcond

            void attemptNewEdge(const PathInfo &p, const PathInfo &q, int indexP, int indexQ);

            base::SpaceInformationPtr si_;
            base::OptimizationObjectivePtr obj_;
            HGraph g_;
            boost::property_map<HGraph, vertex_state_t>::type stateProperty_;
            Vertex root_;
            Vertex goal_;
            std::set<PathInfo> paths_;
            geometric::PathGeometricPtr hpath_;

            /** \brief The name of the path hybridization algorithm, used for tracking planner solution sources */
            std::string name_;
        };
    }
}

#endif
