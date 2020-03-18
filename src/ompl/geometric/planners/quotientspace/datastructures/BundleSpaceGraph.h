/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, University of Stuttgart
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
 *   * Neither the name of the University of Stuttgart nor the names
 *     of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written
 *     permission.
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

/* Author: Andreas Orthey */

#ifndef OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_BUNDLEGRAPH_
#define OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_BUNDLEGRAPH_

#include "BundleSpace.h"
#include <limits>
#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/base/Cost.h>
#include <ompl/datastructures/PDF.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <boost/pending/disjoint_sets.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
// #include <boost/graph/subgraph.hpp>
#include <boost/graph/properties.hpp>
#include <boost/random/linear_congruential.hpp>
#include <boost/random/variate_generator.hpp>

namespace ompl
{
    namespace base
    {
        const double dInf = std::numeric_limits<double>::infinity();
        OMPL_CLASS_FORWARD(OptimizationObjective);
    }
    namespace geometric
    {
        OMPL_CLASS_FORWARD(BundleSpaceImportance);
        OMPL_CLASS_FORWARD(BundleSpaceGraphSampler);
    }

    namespace geometric
    {
        /** \brief A graph on a Bundle-space */
        class BundleSpaceGraph : public BundleSpace
        {
            using BaseT = BundleSpace;

        public:
            using normalized_index_type = int;

            /** \brief A configuration in Bundle-space */
            class Configuration
            {
            public:
                Configuration() = delete;
                Configuration(const ompl::base::SpaceInformationPtr &si);
                Configuration(const ompl::base::SpaceInformationPtr &si, const ompl::base::State *state_);
                ompl::base::State *state{nullptr};
                unsigned int total_connection_attempts{0};
                unsigned int successful_connection_attempts{0};
                bool on_shortest_path{false};

                /** \brief Element of Probability Density Function (needed to update
                     probability) */
                void *pdf_element;
                void setPDFElement(void *element_)
                {
                    pdf_element = element_;
                }
                void *getPDFElement()
                {
                    return pdf_element;
                }

                bool isStart{false};
                bool isGoal{false};

                /** \brief parent index for {qrrt*} */
                Configuration *parent{nullptr};

                /** \brief cost to reach until current vertex in {qrrt*} */
                base::Cost cost;

                /** \brief same as rrt*, connection cost with parent {qrrt*} */
                base::Cost lineCost;

                /** \brief The set of motions descending from the current motion {qrrt*} */
                std::vector<Configuration *> children;

                /** \brief Index of configuration in boost::graph. Usually in
                    the interval [0,num_vertices(graph)], but if vertices are
                    deleted or graphs are copied, we sometimes need to map them
                    back to [0, num_vertices(graph)]  */
                normalized_index_type index{-1};

                /** \brief Access to the representatives (Sparse Vertex) of the Dense vertices
                 * For Sparse Graph: Store index of Sparse Vertex which is represtative of Dense Graph Vertex
                 */
                normalized_index_type representativeIndex{-1};
                
                /** \brief Access to all non-interface supporting vertices of the sparse nodes */
                //boost::property<vertex_list_t, std::set<VertexIndexType>,
                std::set<normalized_index_type> nonInterfaceIndexList;

                /** \brief Access to the interface-supporting vertice hashes of the sparse nodes */
                //boost::property<vertex_interface_list_t, std::unordered_map<VertexIndexType, std::set<VertexIndexType>>>
                std::unordered_map<normalized_index_type, std::set<normalized_index_type>> interfaceIndexList;
            };

            /** \brief An edge in Bundle-space */
            class EdgeInternalState
            {
            public:
                EdgeInternalState() = default;
                EdgeInternalState(ompl::base::Cost cost_) : cost(cost_){};
                EdgeInternalState(const EdgeInternalState &eis)
                {
                    cost = eis.cost;
                }
                void setWeight(double d)
                {
                    cost = ompl::base::Cost(d);
                }
                ompl::base::Cost getCost()
                {
                    return cost;
                }

            private:
                ompl::base::Cost cost{+ompl::base::dInf};
            };

            struct GraphMetaData
            {
                std::string name{"BundleSpaceGraph"};
            };

            /** \brief A Bundle-graph structure using boost::adjacency_list bundles
             *
             * https://www.boost.org/doc/libs/1_71_0/libs/graph/doc/adjacency_list.html
             * */
            using Graph = boost::adjacency_list<
                  boost::vecS, 
                  boost::vecS, 
                  boost::undirectedS, 
                  Configuration*,
                  EdgeInternalState, 
                  GraphMetaData
            >;
            using BGT = boost::graph_traits<Graph>;
            using Vertex = BGT::vertex_descriptor;
            using Edge = BGT::edge_descriptor;
            using VertexIndex = BGT::vertices_size_type;
            using IEIterator = BGT::in_edge_iterator;
            using OEIterator = BGT::out_edge_iterator;
            using VertexParent = Vertex;
            using VertexRank = VertexIndex;
            using RoadmapNeighborsPtr = std::shared_ptr<NearestNeighbors<Configuration *>>;
            using PDF = ompl::PDF<Configuration *>;
            using PDF_Element = PDF::Element;

        public:
            BundleSpaceGraph(const ompl::base::SpaceInformationPtr &si, BundleSpace *parent = nullptr);
            virtual ~BundleSpaceGraph();

            virtual unsigned int getNumberOfVertices() const;
            virtual unsigned int getNumberOfEdges() const;

            virtual void grow() = 0;
            virtual void sampleFromDatastructure(ompl::base::State *) override;
            virtual void sampleBundleGoalBias(ompl::base::State *xRandom, double goalBias);
            virtual bool getSolution(ompl::base::PathPtr &solution) override;

            /** \brief Return plannerdata structure, whereby each vertex is marked
                depending to which component it belongs (start/goal/non-connected) */
            virtual void getPlannerData(ompl::base::PlannerData &data) const override;
            void getPlannerDataGraph(ompl::base::PlannerData &data, 
                const Graph &graph, const Vertex vStart, const Vertex vGoal) const;

            /** \brief Importance of Bundle-space depending on number of
                vertices in Bundle-graph */
            virtual double getImportance() const override;

            /** \brief Initialization methods for the first iteration
                 (adding start configuration and doing sanity checks) */
            void init();

            virtual void setup() override;
            virtual void clear() override;
            void clearQuery();
            virtual void clearVertices();
            virtual void deleteConfiguration(Configuration *q);

            template <template <typename T> class NN>
            void setNearestNeighbors();
            void uniteComponents(Vertex m1, Vertex m2);
            bool sameComponent(Vertex m1, Vertex m2);
            std::map<Vertex, VertexRank> vrank;
            std::map<Vertex, Vertex> vparent;
            boost::disjoint_sets<boost::associative_property_map<std::map<Vertex, VertexRank>>,
                                 boost::associative_property_map<std::map<Vertex, Vertex>>>
                disjointSets_{boost::make_assoc_property_map(vrank), boost::make_assoc_property_map(vparent)};

            virtual const Configuration *nearest(const Configuration *s) const;

            virtual void setMetric(const std::string& sMetric) override;
            virtual void setPropagator(const std::string& sPropagator) override;
            virtual void setImportance(const std::string& sImportance);
            virtual void setGraphSampler(const std::string& sGraphSampler);

            ompl::base::Cost bestCost_{+ompl::base::dInf};
            Configuration *qStart_{nullptr};
            Configuration *qGoal_{nullptr};
            Vertex vStart_;
            Vertex vGoal_;

            std::vector<Vertex> shortestVertexPath_;
            std::vector<Vertex> startGoalVertexPath_;
            double lengthStartGoalVertexPath_;
            std::vector<double> lengthsStartGoalVertexPath_;


            const Graph &getGraph() const;
            double getGraphLength() const;
            const RoadmapNeighborsPtr &getRoadmapNeighborsPtr() const;

            virtual void print(std::ostream &out) const override;
            /** \brief Print configuration to std::cout */
            void printConfiguration(const Configuration *) const;

            void getPathDenseGraphPath(const Vertex &start, const Vertex &goal, Graph &graph, std::deque<base::State *> &path);

            void setGoalBias(double goalBias);

            double getGoalBias() const;

            void setRange(double distance);

            double getRange() const;

            /** \brief Shortest path on Bundle-graph */
            ompl::base::PathPtr getPath(const Vertex &start, const Vertex &goal);
            ompl::base::PathPtr getPath(const Vertex &start, const Vertex &goal, Graph &graph);

            virtual double distance(const Configuration *a, const Configuration *b) const;
            virtual bool checkMotion(const Configuration *a, const Configuration *b) const;

            const Configuration* extendGraphTowards(
                const Configuration *from, 
                const Configuration *to);

            bool connect(
                const Configuration *from, 
                const Configuration *to);

            Configuration* extendGraphTowards_Range(
                const Configuration *from, 
                const Configuration *to);

            virtual void interpolate(
                const Configuration *a, 
                const Configuration *b, 
                Configuration *dest) const;

            ompl::base::PathPtr solutionPath_;
        protected:

            virtual Configuration* addBundleConfiguration(base::State*);
            virtual void addBundleEdge(const Configuration *a, const Configuration *b);

            virtual Vertex addConfiguration(Configuration *q);
            void addEdge(const Vertex a, const Vertex b);

            ompl::base::Cost costHeuristic(Vertex u, Vertex v) const;

            /** \brief Nearest neighbor structure for Bundle space configurations */
            RoadmapNeighborsPtr nearestDatastructure_;
            Graph graph_;
            unsigned int numVerticesWhenComputingSolutionPath{0};
            RNG rng_;
            using RNGType = boost::minstd_rand;
            RNGType rng_boost;

            /** \brief Length of graph (useful for determing importance of
                Bundle-space */
            double graphLength_{0.0};

            /** \brief Maximum expansion step */
            double maxDistance_{-1.0};

            /** \brief Goal bias */
            double goalBias_{.05};

            /** \brief Temporary random configuration */
            Configuration *xRandom_{nullptr};

            /** \brief Temporary random configuration */
            Configuration *xApproximateNearest_{nullptr};

            double bestDist_{base::dInf};

            BundleSpaceImportancePtr importanceCalculator_;

            BundleSpaceGraphSamplerPtr graphSampler_;

            ompl::base::OptimizationObjectivePtr pathRefinementObj_;
        };
    }  // namespace geometric
}  // namespace ompl

#endif
