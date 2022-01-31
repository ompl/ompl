/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, University of Stuttgart.
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

/* Author: Andreas Orthey, Sohaib Akbar */

#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_BUNDLEGRAPH_
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_BUNDLEGRAPH_

#include <ompl/multilevel/datastructures/BundleSpace.h>
#include <ompl/multilevel/datastructures/pathrestriction/FindSectionTypes.h>
#include <limits>
#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/base/Cost.h>
#include <ompl/control/Control.h>
#include <ompl/datastructures/PDF.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <boost/pending/disjoint_sets.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
// #include <boost/graph/subgraph.hpp> //Note: Everything would be nicer with
// subgraphs, but there are still some bugs in the boost impl which prevent
// us to use them here
#include <boost/graph/properties.hpp>
#include <boost/random/linear_congruential.hpp>
#include <boost/random/variate_generator.hpp>

namespace ompl
{
    namespace base
    {
        const double dInf = std::numeric_limits<double>::infinity();
    }
    namespace multilevel
    {
        /// @cond IGNORE
        /** \brief Forward declaration of * ompl::multilevel::BundleSpaceImportance */
        OMPL_CLASS_FORWARD(BundleSpaceImportance);
        /** \brief Forward declaration of * ompl::multilevel::BundleSpaceGraphSampler */
        OMPL_CLASS_FORWARD(BundleSpaceGraphSampler);
        /** \brief Forward declaration of * ompl::multilevel::PathRestriction */
        OMPL_CLASS_FORWARD(PathRestriction);
        /// @endcond
    }
    namespace geometric
    {
        OMPL_CLASS_FORWARD(PathSimplifier);
    }

    namespace multilevel
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
                ompl::control::Control *control{nullptr};

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
                // boost::property<vertex_list_t, std::set<VertexIndexType>,
                std::set<normalized_index_type> nonInterfaceIndexList;

                /** \brief Access to the interface-supporting vertice hashes of the sparse nodes */
                // boost::property<vertex_interface_list_t, std::unordered_map<VertexIndexType,
                // std::set<VertexIndexType>>>
                std::unordered_map<normalized_index_type, std::set<normalized_index_type>> interfaceIndexList;

                std::vector<Configuration *> reachableSet;
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
                EdgeInternalState& operator=(const EdgeInternalState &eis)
                {
                    cost = eis.cost;
                    return *this;
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
            using Graph = boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, Configuration *,
                                                EdgeInternalState, GraphMetaData>;
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

            /* \brief Number of vertices on boost graph */
            virtual unsigned int getNumberOfVertices() const;

            /* \brief Number of edges on boost graph */
            virtual unsigned int getNumberOfEdges() const;

            /* \brief A null vertex representing a non-existing vertex */
            Vertex nullVertex() const;

            /* \brief One iteration of the growing the graph */
            void grow() override = 0;

            /* \brief Sample from the graph (used in restriction sampling for
             * parent bundle space) */
            void sampleFromDatastructure(ompl::base::State *) override;

            /* \brief as sampleBundle() but with goal bias */
            virtual void sampleBundleGoalBias(ompl::base::State *xRandom);

            /* \brief Check that there exist a path on graph */
            bool getSolution(ompl::base::PathPtr &solution) override;

            /* \brief Return best cost path on graph (as reference) */
            virtual ompl::base::PathPtr &getSolutionPathByReference();

            /** \brief Return plannerdata structure, whereby each vertex is marked
                depending to which component it belongs (start/goal/non-connected) */
            void getPlannerData(ompl::base::PlannerData &data) const override;

            /* \brief Given graph, fill in the ompl::base::PlannerData structure */
            void getPlannerDataGraph(ompl::base::PlannerData &data, const Graph &graph, const Vertex vStart) const;

            /** \brief Importance of Bundle-space depending on number of
                vertices in Bundle-graph */
            double getImportance() const override;

            /** \brief Initialization methods for the first iteration
                 (adding start configuration and doing sanity checks) */
            virtual void init();

            void setup() override;
            void clear() override;

            /* \brief Delete all vertices and their attached configurations */
            virtual void clearVertices();

            /* \brief Delete a configuration and free its state */
            virtual void deleteConfiguration(Configuration *q);

            /* \brief Create nearest neighbors structure */
            template <template <typename T> class NN>
            void setNearestNeighbors();

            /* \brief Unite two components */
            void uniteComponents(Vertex m1, Vertex m2);
            /* \brief Check if both vertices are in the same component */
            bool sameComponent(Vertex m1, Vertex m2);
            std::map<Vertex, VertexRank> vrank;
            std::map<Vertex, Vertex> vparent;
            boost::disjoint_sets<boost::associative_property_map<std::map<Vertex, VertexRank>>,
                                 boost::associative_property_map<std::map<Vertex, Vertex>>>
                disjointSets_{boost::make_assoc_property_map(vrank), boost::make_assoc_property_map(vparent)};

            /* \brief Get nearest configuration to configuration s */
            virtual const Configuration *nearest(const Configuration *s) const;

            /* \brief Set metric to be used for growing graph */
            void setMetric(const std::string &sMetric) override;
            void setPropagator(const std::string &sPropagator) override;
            virtual void setImportance(const std::string &sImportance);
            virtual void setGraphSampler(const std::string &sGraphSampler);

            /* \brief Set strategy to solve the find section problem */
            virtual void setFindSectionStrategy(FindSectionType type);

            /* \brief Get current graph sampler */
            BundleSpaceGraphSamplerPtr getGraphSampler();

            /** \brief Best cost found so far by algorithm */
            base::Cost bestCost_{+base::dInf};

            /* \brief Best cost path on graph as vertex representation */
            std::vector<Vertex> shortestVertexPath_;

            /** \brief Get underlying boost graph representation (non const)*/
            virtual Graph &getGraphNonConst();
            /** \brief Get underlying boost graph representation */
            virtual const Graph &getGraph() const;

            const RoadmapNeighborsPtr &getRoadmapNeighborsPtr() const;

            /** \brief Print class to ostream */
            void print(std::ostream &out) const override;

            /** \brief Write class to graphviz */
            void writeToGraphviz(std::string filename) const;

            /** \brief Print configuration to std::cout */
            virtual void printConfiguration(const Configuration *) const;

            void setGoalBias(double goalBias);

            double getGoalBias() const;

            void setRange(double distance);

            double getRange() const;

            /** \brief Shortest path on Bundle-graph */
            ompl::base::PathPtr getPath(const Vertex &start, const Vertex &goal);
            ompl::base::PathPtr getPath(const Vertex &start, const Vertex &goal, Graph &graph);

            /** \brief Distance between two configurations using the current
             * metric. */
            virtual double distance(const Configuration *a, const Configuration *b) const;
            /** \brief Check if we can move from configuration a to
             * configuration b using the current metric. */
            virtual bool checkMotion(const Configuration *a, const Configuration *b) const;

            /** \brief Try to connect configuration a to
             * configuration b using the current metric. */
            bool connect(const Configuration *from, const Configuration *to);

            /** \brief Steer system at Configuration *from to Configuration
             * *to */
            Configuration *steerTowards(const Configuration *from, const Configuration *to);

            /** \brief Steer system at Configuration *from to Configuration
             * *to, stopping if maxdistance is reached */
            Configuration *steerTowards_Range(const Configuration *from, Configuration *to);

            /** \brief Steer system at Configuration *from to Configuration
             * *to while system is valid, stopping if maxDistance is reached */
            Configuration *extendGraphTowards_Range(const Configuration *from, Configuration *to);

            /** \brief Interpolate from configuration a to configuration b and
             * store results in dest */
            virtual void interpolate(const Configuration *a, const Configuration *b, Configuration *dest) const;

            /** \brief Add ompl::base::State to graph. Return its configuration. */
            virtual Configuration *addBundleConfiguration(base::State *);

            /** \brief Add configuration to graph. Return its vertex in boost
             * graph */
            virtual Vertex addConfiguration(Configuration *q);
            /** \brief Add configuration to graph as goal vertex. */
            void addGoalConfiguration(Configuration *x);

            /** \brief Add edge between configuration a and configuration b to graph. */
            virtual void addBundleEdge(const Configuration *a, const Configuration *b);

            /** \brief Add edge between Vertex a and Vertex b to graph. */
            virtual const std::pair<Edge, bool> addEdge(const Vertex a, const Vertex b);

            /** \brief Get vertex representing the start. */
            virtual Vertex getStartIndex() const;
            /** \brief Get vertex representing the goal. */
            virtual Vertex getGoalIndex() const;
            /** \brief Set vertex representing the start. */
            virtual void setStartIndex(Vertex);

            /**\brief Call algorithm to solve the find section problem */
            bool findSection() override;

        protected:
            ompl::base::PathPtr solutionPath_;

            Vertex vStart_;

            ompl::base::Cost costHeuristic(Vertex u, Vertex v) const;

            /** \brief Nearest neighbor structure for Bundle space configurations */
            RoadmapNeighborsPtr nearestDatastructure_;
            Graph graph_;
            unsigned int numVerticesWhenComputingSolutionPath_{0};
            RNG rng_;
            using RNGType = boost::minstd_rand;
            RNGType rng_boost;

            /** \brief Length of graph (useful for determing importance of
                Bundle-space */
            double graphLength_{0.0};

            /** \brief Maximum expansion step */
            double maxDistance_{-1.0};

            /** \brief Goal bias */
            double goalBias_{.1};

            /** \brief Temporary random configuration */
            Configuration *xRandom_{nullptr};

            /** \brief Pointer to strategy to compute importance of this bundle
             * space (which is used to decide which bundle space to grow next)
             * */
            BundleSpaceImportancePtr importanceCalculator_{nullptr};

            /** \brief Pointer to strategy to sample from graph */
            BundleSpaceGraphSamplerPtr graphSampler_{nullptr};

            /** \brief Pointer to current path restriction (the set of points
             * which project onto the best cost path on the base space if any).
             * This only exists if there exists a base space and there exists a
             * base space path. */
            PathRestrictionPtr pathRestriction_{nullptr};

            /** \brief A path optimizer */
            ompl::geometric::PathSimplifierPtr optimizer_;

            /** \brief Start configuration */
            Configuration *qStart_{nullptr};

            /** \brief The (best) goal configuration */
            Configuration *qGoal_{nullptr};

            /** \brief List of configurations that satisfy the start condition */
            std::vector<Configuration *> startConfigurations_;

            /** \brief List of configurations that satisfy the goal condition */
            std::vector<Configuration *> goalConfigurations_;
        };
    }  // namespace multilevel
}  // namespace ompl

#endif
