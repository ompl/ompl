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

/* Author: Oren Salzman */
/* Implementation based on
Sven Koenig, Maxim Likhachev, David Furcy:
Lifelong Planning A. Artif. Intell. 155(1-2): 93-146 (2004)
*/

#ifndef OMPL_DATASTRUCTURES_LPA_STAR_ON_G_H
#define OMPL_DATASTRUCTURES_LPA_STAR_ON_G_H

#include <vector>
#include <limits>
#include <set>
#include <map>
#include <list>
#include <unordered_map>

#include <iterator>
#include <iostream>
#include <cassert>

// workaround for bug in Boost 1.60; see https://svn.boost.org/trac/boost/ticket/11880
#include <boost/version.hpp>
#if BOOST_VERSION == 106000
#include <boost/type_traits/ice.hpp>
#endif

#include <boost/functional/hash.hpp> // fix for Boost < 1.68
#include <boost/graph/adjacency_matrix.hpp>
#include <boost/graph/adjacency_list.hpp>

namespace ompl
{
    // Data is of type std::size_t
    template <typename Graph,      // Boost graph
              typename Heuristic>  // heuristic to estimate cost
    class LPAstarOnGraph
    {
    public:
        LPAstarOnGraph(std::size_t source, std::size_t target, Graph &graph, Heuristic &h)
          : costEstimator_(h), graph_(graph)
        {
            // initialization
            double c = std::numeric_limits<double>::infinity();
            source_ = new Node(c, costEstimator_(source), 0, source);
            addNewNode(source_);
            target_ = new Node(c, 0, c, target);
            addNewNode(target_);
            insertQueue(source_);
        }
        ~LPAstarOnGraph()
        {
            clear();
        }

        void insertEdge(std::size_t u, std::size_t v, double c)
        {
            Node *n_u = getNode(u);
            Node *n_v = getNode(v);

            if (n_v->rhs() > n_u->costToCome() + c)
            {
                n_v->setParent(n_u);
                n_v->setRhs(n_u->costToCome() + c);
                updateVertex(n_v);
            }
        }
        void removeEdge(std::size_t u, std::size_t v)
        {
            assert(v != source_->getId());

            Node *n_u = getNode(u);
            Node *n_v = getNode(v);

            if (n_v->getParent() == n_u)
            {
                WeightMap weights = boost::get(boost::edge_weight_t(), graph_);
                chooseBestIncomingNode(n_v, weights);
            }

            updateVertex(n_v);
        }
        double computeShortestPath(std::list<std::size_t> &path)
        {
            WeightMap weights = boost::get(boost::edge_weight_t(), graph_);

            if (queue_.empty())
                return std::numeric_limits<double>::infinity();

            while (topHead()->key() < target_->calculateKey() || target_->rhs() != target_->costToCome())
            {
                // pop from queue and process
                Node *u = topHead();

                if (u->costToCome() > u->rhs())  // the node is overconsistent
                {
                    u->setCostToCome(u->rhs());
                    popHead();

                    // iterate over all (outgoing) neighbors of the node and get the best parent for each one
                    typename boost::graph_traits<Graph>::out_edge_iterator ei, ei_end;
                    for (boost::tie(ei, ei_end) = boost::out_edges(u->getId(), graph_); ei != ei_end; ++ei)
                    {
                        std::size_t v = boost::target(*ei, graph_);
                        Node *n_v = getNode(v);
                        double c = boost::get(weights, *ei);  // edge weight from u to v

                        if (n_v->rhs() > u->costToCome() + c)
                        {
                            n_v->setParent(u);
                            n_v->setRhs(u->costToCome() + c);
                            updateVertex(n_v);
                        }
                    }
                }
                else  // (n->costToCome() < n->rhs()) // the node is underconsistent
                {
                    u->setCostToCome(std::numeric_limits<double>::infinity());
                    updateVertex(u);

                    // get all (outgoing) neighbors of the node
                    typename boost::graph_traits<Graph>::out_edge_iterator ei, ei_end;
                    for (boost::tie(ei, ei_end) = boost::out_edges(u->getId(), graph_); ei != ei_end; ++ei)
                    {
                        std::size_t v = boost::target(*ei, graph_);
                        Node *n_v = getNode(v);

                        if ((n_v == source_) || (n_v->getParent() != u))
                            continue;

                        chooseBestIncomingNode(n_v, weights);
                        updateVertex(n_v);
                    }
                }

                if (queue_.empty())
                    break;
            }

            // now get path
            Node *res = (target_->costToCome() == std::numeric_limits<double>::infinity() ? nullptr : target_);
            while (res != nullptr)
            {
                path.push_front(res->getId());
                res = res->getParent();
            }

            return target_->costToCome();
        }

        /// using LPA* to approximate costToCome
        double operator()(std::size_t u)
        {
            auto iter = idNodeMap_.find(u);
            if (iter != idNodeMap_.end())
                return iter->second->costToCome();
            return std::numeric_limits<double>::infinity();
        }

    private:
        struct Key
        {
            Key(double first_ = -1, double second_ = -1) : first(first_), second(second_)
            {
            }
            bool operator<(const Key &other)
            {
                return (first != other.first) ? (first < other.first) : (second < other.second);
            }
            double first, second;
        };

        class Node
        {
        public:
            Node(double costToCome, double costToGo, double rhs, std::size_t &dataId, Node *parentNode = nullptr)
              : g(costToCome), h(costToGo), r(rhs), isInQ(false), parent(parentNode), id(dataId)
            {
                calculateKey();
            }
            // cost accesors
            double costToCome() const
            {
                return g;
            }
            double costToGo() const
            {
                return h;
            }
            double rhs() const
            {
                return r;
            }
            Key key() const
            {
                return k;
            }
            Key calculateKey()
            {
                k = Key(std::min(g, r + h), std::min(g, r));
                return k;
            }
            // cost modifiers
            double setCostToCome(double val)
            {
                return g = val;
            }
            double setRhs(double val)
            {
                return r = val;
            }
            // is in queue field
            bool isInQueue() const
            {
                return isInQ;
            }
            void inQueue(bool in)
            {
                isInQ = in;
            }
            // parent field
            Node *getParent() const
            {
                return parent;
            }
            void setParent(Node *p)
            {
                parent = p;
            }
            // data field
            std::size_t getId() const
            {
                return id;
            }
            bool isConsistent() const
            {
                return g == r;
            }

        private:
            double g;  // cost to come
            double h;  // cost to go
            double r;  // rhs
            Key k;     // key
            bool isInQ;
            Node *parent;
            std::size_t id;  // unique data associated with node
        };                   // Node

        struct LessThanNodeK
        {
            bool operator()(const Node *n1, const Node *n2) const
            {
                return n1->key() < n2->key();
            }
        };  // LessThanNodeK

        struct Hash
        {
            std::size_t operator()(const std::size_t id) const
            {
                return h(id);
            }
            std::hash<std::size_t> h;
        };  // Hash

        using Queue = std::multiset<Node *, LessThanNodeK>;
        using IdNodeMap = std::unordered_map<std::size_t, Node *, Hash>;
        using IdNodeMapIter = typename IdNodeMap::iterator;
        using WeightMap = typename boost::property_map<Graph, boost::edge_weight_t>::type;

        // LPA* subprocedures
        void updateVertex(Node *n)
        {
            if (!n->isConsistent())
            {
                if (n->isInQueue())
                    updateQueue(n);
                else
                    insertQueue(n);
            }
            else if (n->isInQueue())
                removeQueue(n);
        }
        // queue utils
        Node *popHead()
        {
            Node *n = topHead();
            n->inQueue(false);
            queue_.erase(queue_.begin());

            return n;
        }
        Node *topHead()
        {
            return *queue_.begin();
        }

        void insertQueue(Node *node)
        {
            assert(node->isInQueue() == false);

            node->calculateKey();
            node->inQueue(true);
            queue_.insert(node);
       }
        void removeQueue(Node *node)
        {
            if (node->isInQueue())
            {
                node->inQueue(false);
                queue_.erase(node);
            }
        }
        void updateQueue(Node *node)
        {
            removeQueue(node);
            insertQueue(node);
        }

        void chooseBestIncomingNode(Node *n_v, WeightMap &weights)
        {
            // iterate over all incoming neighbors of the node n_v and get the best parent
            double min = std::numeric_limits<double>::infinity();
            Node *best = nullptr;

            typename boost::graph_traits<Graph>::in_edge_iterator ei, ei_end;
            for (boost::tie(ei, ei_end) = boost::in_edges(n_v->getId(), graph_); ei != ei_end; ++ei)
            {
                std::size_t u = boost::source(*ei, graph_);
                Node *n_u = getNode(u);
                double c = boost::get(weights, *ei);  // edge weight from u to v

                double curr = n_u->costToCome() + c;
                if (curr < min)
                {
                    min = curr;
                    best = n_u;
                }
            }

            n_v->setRhs(min);
            n_v->setParent(best);
        }

        void addNewNode(Node *n)
        {
            idNodeMap_[n->getId()] = n;
        }

        Node *getNode(std::size_t id)
        {
            auto iter = idNodeMap_.find(id);
            if (iter != idNodeMap_.end())
                return iter->second;

            double c = std::numeric_limits<double>::infinity();
            auto *n = new Node(c, costEstimator_(id), c, id);
            addNewNode(n);

            return n;
        }

        void clear()
        {
            for (auto &id : idNodeMap_)
                delete id.second;
        }

        Heuristic &costEstimator_;
        Graph &graph_;
        Node *source_;
        Node *target_;
        Queue queue_;
        IdNodeMap idNodeMap_;

    };  // LPAstarOnGraph
}

#endif  // OMPL_DATASTRUCTURES_LPA_STAR_ON_G_H