/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Tel Aviv University
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
 *   * Neither the name of the Tel Aviv University nor the names of its
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

/* Author: Oren Salzman, Aditya Mandalika */
/* Implementation based on
G. Ramalingam and T. W. Reps, On the computational complexity of
dynamic graph problems, Theor. Comput. Sci., vol. 158, no. 1&2, pp.
233-277, 1996.
*/

#ifndef OMPL_DATASTRUCTURES_DYNAMICSSSP_H
#define OMPL_DATASTRUCTURES_DYNAMICSSSP_H

#include <list>
#include <set>
#include <vector>
#include <limits>

#include <boost/functional/hash.hpp> // fix for Boost < 1.68
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <unordered_set>

namespace ompl
{
    class DynamicSSSP
    {
    public:
        DynamicSSSP()
        {
            graph_ = new Graph(0);
        }
        ~DynamicSSSP()
        {
            delete graph_;
        }

        void clear()
        {
            graph_->clear();
            distance_.clear();
            parent_.clear();
        }

        void addVertex(std::size_t id)
        {
            distance_.push_back((id == 0) ? 0 : std::numeric_limits<double>::infinity());
            parent_.push_back(NO_ID);
            boost::add_vertex(id, *graph_);
        }

        // we assume that no two paths have the same cost,
        // this asssumption is valid when the nodes have some randomeness to them
        void addEdge(std::size_t v, std::size_t w, double weight, bool collectVertices,
                     std::list<std::size_t> &affectedVertices)
        {
            // first, add edge to graph
            WeightProperty edge_property(weight);
            boost::add_edge(v, w, edge_property, *graph_);

            // now, update distance_
            assert((distance_[v] == std::numeric_limits<double>::infinity()) ||
                   (distance_[w] == std::numeric_limits<double>::infinity()) ||
                   (distance_[w] + weight != distance_[w]));

            std::vector<double> cost(boost::num_vertices(*graph_),
                                     std::numeric_limits<double>::infinity());  // initialize to n values of cost oo

            IsLessThan isLessThan(cost);
            Queue queue(isLessThan);

            if (distance_[v] + weight < distance_[w])
            {
                distance_[w] = distance_[v] + weight;
                parent_[w] = v;

                cost[w] = 0;
                queue.insert(w);
            }

            WeightMap weights = boost::get(boost::edge_weight_t(), *graph_);
            while (!queue.empty())
            {
                // pop head of queue
                std::size_t u = *(queue.begin());
                queue.erase(queue.begin());

                if (collectVertices)
                    affectedVertices.push_back(u);

                boost::out_edges(u, *graph_);

                // for every outgoing edge, see if we can improve its cost
                boost::graph_traits<Graph>::out_edge_iterator ei, ei_end;
                for (boost::tie(ei, ei_end) = boost::out_edges(u, *graph_); ei != ei_end; ++ei)
                {
                    std::size_t x = boost::target(*ei, *graph_);
                    double edgeWeight = boost::get(weights, *ei);

                    if (distance_[u] + edgeWeight < distance_[x])
                    {
                        distance_[x] = distance_[u] + edgeWeight;
                        parent_[x] = u;

                        // insert to queue
                        auto qIter = queue.find(x);
                        if (qIter != queue.end())
                            queue.erase(qIter);

                        cost[x] = distance_[x] - distance_[v];
                        queue.insert(x);
                    }
                }
            }
        }

        void removeEdge(std::size_t v, std::size_t w, bool collectVertices, std::list<std::size_t> &affectedVertices)
        {
            // first, remove edge from graph
            boost::remove_edge(v, w, *graph_);
            if (parent_[w] != v)
                return;

            // Phase 1: Identify the affected vertices and remove the affected edges from SP(G)
            std::list<std::size_t> workSet;
            IntSet affectedVerticesSet;
            workSet.push_back(w);

            while (!workSet.empty())
            {
                // S elect and remove a vertex u from WorkSet
                std::size_t u = workSet.front();
                workSet.pop_front();

                affectedVerticesSet.insert(u);

                boost::graph_traits<Graph>::out_edge_iterator ei, ei_end;
                for (boost::tie(ei, ei_end) = boost::out_edges(u, *graph_); ei != ei_end; ++ei)
                {
                    std::size_t x = boost::target(*ei, *graph_);
                    if (parent_[x] == u)
                        workSet.push_back(x);
                }
            }

            WeightMap weights = boost::get(boost::edge_weight_t(), *graph_);

            // Phase 2: Determine new distances from affected vertices to source(G) and update SP(G).
            IsLessThan isLessThan(distance_);
            Queue queue(isLessThan);
            for (auto set_iter = affectedVerticesSet.begin(); set_iter != affectedVerticesSet.end(); ++set_iter)
            {
                std::size_t a = *set_iter;
                distance_[a] = std::numeric_limits<double>::infinity();

                // go over all incoming neighbors which are NOT affected vertices
                // get the best such neighbor
                boost::graph_traits<Graph>::in_edge_iterator ei, ei_end;
                for (boost::tie(ei, ei_end) = boost::in_edges(a, *graph_); ei != ei_end; ++ei)
                {
                    std::size_t b = boost::source(*ei, *graph_);
                    if (affectedVerticesSet.find(b) == affectedVerticesSet.end())
                    {
                        double edgeWeight = boost::get(weights, *ei);

                        if (distance_[b] + edgeWeight < distance_[a])
                        {
                            distance_[a] = distance_[b] + edgeWeight;
                            parent_[a] = b;
                        }
                    }
                }
                if (distance_[a] != std::numeric_limits<double>::infinity())
                    queue.insert(a);
            }

            while (!queue.empty())
            {
                // pop head of queue
                std::size_t a = *queue.begin();
                queue.erase(queue.begin());

                if (collectVertices)
                    affectedVertices.push_back(a);

                // for every outgoing edge, see if we can improve its cost
                boost::graph_traits<Graph>::out_edge_iterator ei, ei_end;
                for (boost::tie(ei, ei_end) = boost::out_edges(a, *graph_); ei != ei_end; ++ei)
                {
                    int c = boost::target(*ei, *graph_);
                    double edgeWeight = boost::get(weights, *ei);

                    if (distance_[a] + edgeWeight < distance_[c])
                    {
                        distance_[c] = distance_[a] + edgeWeight;
                        parent_[c] = a;

                        // insert to queue
                        auto qIter = queue.find(c);
                        if (qIter != queue.end())
                            queue.erase(qIter);

                        queue.insert(c);
                    }
                }
            }
        }

        double getShortestPathCost(std::size_t u) const
        {
            return this->distance_[u];
        }

        std::size_t getShortestPathParent(std::size_t u) const
        {
            return parent_[u];
        }

    private:
        using WeightProperty = boost::property<boost::edge_weight_t, double>;
        using Graph = boost::adjacency_list<boost::vecS,            // container type for the edge list
                                            boost::vecS,            // container type for the vertex list
                                            boost::bidirectionalS,  // directedS / undirectedS / bidirectionalS
                                            std::size_t,            // vertex properties
                                            WeightProperty>;        // edge properties
        using WeightMap = boost::property_map<Graph, boost::edge_weight_t>::type;

        static const int NO_ID = -1;

        class IsLessThan
        {
        public:
            IsLessThan(std::vector<double> &cost) : cost_(cost)
            {
            }

            bool operator()(std::size_t id1, std::size_t id2) const
            {
                return (cost_[id1] < cost_[id2]);
            }

        private:
            std::vector<double> &cost_;
        };  // IsLessThan

        using Queue = std::set<std::size_t, IsLessThan>;
        using QueueIter = Queue::iterator;
        using IntSet = std::unordered_set<std::size_t>;
        using IntSetIter = IntSet::iterator;

        Graph *graph_;
        /// \brief distance from source which is node zero
        std::vector<double> distance_;
        /// \brief parent of each node
        std::vector<std::size_t> parent_;
    };  // DynamicSSSP
}  // namespace ompl

#endif  // OMPL_DATASTRUCTURES_DYNAMICSSSP_H
