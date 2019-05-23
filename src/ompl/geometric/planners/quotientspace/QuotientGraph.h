
#ifndef OMPL_GEOMETRIC_PLANNERS_QUOTIENTSPACE_QUOTIENTGRAPH_
#define OMPL_GEOMETRIC_PLANNERS_QUOTIENTSPACE_QUOTIENTGRAPH_

#include "Quotient.h"
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
#include <boost/graph/subgraph.hpp>
#include <boost/graph/properties.hpp>
#include <boost/random/linear_congruential.hpp>
#include <boost/random/variate_generator.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;
const double dInf = std::numeric_limits<double>::infinity();

namespace ompl 
{
    namespace base 
    {
        OMPL_CLASS_FORWARD(OptimizationObjective);
    }
    namespace geometric {

        /// \brief A graph on a quotient-space
        class QuotientGraph : public og::Quotient {
            typedef og::Quotient BaseT;

        public:
            typedef int normalized_index_type;

            /// A configuration in quotient-space 
            class Configuration {
            public:
                Configuration() = delete;
                Configuration(const ob::SpaceInformationPtr &si);
                Configuration(const ob::SpaceInformationPtr &si, const ob::State *state_);
                ob::State *state{nullptr};
                uint total_connection_attempts{0};
                uint successful_connection_attempts{0};
                bool on_shortest_path{false};

                void *pdf_element;
                void setPDFElement(void *element_) { pdf_element = element_; }
                void *getPDFElement() { return pdf_element; }

                unsigned long int associated_target{0};
                unsigned long int associated_source{0};
                double associated_t{-1};

                bool isStart{false};
                bool isGoal{false};
                bool isFeasible{false};

                normalized_index_type index{-1}; // in [0,num_vertices(graph)]
            };

            /// An edge in quotient-space 
            class EdgeInternalState {
            public:
                EdgeInternalState() = default;
                EdgeInternalState(ob::Cost cost_) : cost(cost_), original_cost(cost_){};
                EdgeInternalState(const EdgeInternalState &eis) {
                    cost = eis.cost;
                    original_cost = eis.original_cost;
                }
                void setWeight(double d) { cost = ob::Cost(d); }
                ob::Cost getCost() { return cost; }
                void setOriginalWeight() { cost = original_cost; }

            private:
                ob::Cost cost{+dInf};
                ob::Cost original_cost{+dInf};
            };

            struct GraphBundle {
                std::string name{"quotient_graph"};
            };
            /// A quotient-graph structure using boost::adjacency_list bundles
            typedef boost::adjacency_list<
                boost::vecS, 
                boost::vecS, 
                boost::undirectedS,
                Configuration*, 
                EdgeInternalState, 
                GraphBundle
            > Graph;

            typedef boost::graph_traits<Graph> BGT;
            typedef BGT::vertex_descriptor Vertex;
            typedef BGT::edge_descriptor Edge;
            typedef BGT::vertices_size_type VertexIndex;
            typedef BGT::in_edge_iterator IEIterator;
            typedef BGT::out_edge_iterator OEIterator;
            typedef Vertex *VertexParent;
            typedef VertexIndex *VertexRank;
            typedef std::shared_ptr<NearestNeighbors<Configuration *>>
                    RoadmapNeighborsPtr;
            typedef ompl::PDF<Configuration *> PDF;
            typedef PDF::Element PDF_Element;

        public:
            QuotientGraph(const ob::SpaceInformationPtr &si, Quotient *parent = nullptr);
            ~QuotientGraph();

            virtual uint getNumberOfVertices() const;
            virtual uint getNumberOfEdges() const;

            virtual void grow() = 0;
            virtual bool sampleQuotient(ob::State *) override;
            virtual bool getSolution(ob::PathPtr &solution) override;

            /// \brief Return plannerdata structure, whereby each vertex is marked
            /// depending to which component it belongs (start/goal/non-connected)
            virtual void getPlannerData(ob::PlannerData &data) const override;
            /// \brief Importance of quotient-space depending on number of
            /// vertices in quotient-graph
            virtual double getImportance() const override;
            void init();

            virtual void setup() override;
            virtual void clear() override;
            void clearQuery();
            virtual void clearVertices();
            void deleteConfiguration(Configuration *q);

            template <template <typename T> class NN> void setNearestNeighbors();
            void uniteComponents(Vertex m1, Vertex m2);
            bool sameComponent(Vertex m1, Vertex m2);
            std::map<Vertex, VertexRank> vrank;
            std::map<Vertex, Vertex> vparent;
            boost::disjoint_sets<
                    boost::associative_property_map<std::map<Vertex, VertexRank>>,
                    boost::associative_property_map<std::map<Vertex, Vertex>>> disjointSets_{
                    boost::make_assoc_property_map(vrank),
                    boost::make_assoc_property_map(vparent)};

            const Configuration *nearest(const Configuration *s) const;

            ob::Cost bestCost_{+dInf};
            Configuration *qStart_;
            Configuration *qGoal_;
            Vertex vStart_;
            Vertex vGoal_;
            std::vector<Vertex> shortestVertexPath_;
            std::vector<Vertex> startGoalVertexPath_;

            const Graph &getGraph() const;
            double getGraphLength() const;
            const RoadmapNeighborsPtr &getRoadmapNeighborsPtr() const;

            virtual void print(std::ostream &out) const override;
            /// Print configuration to std::cout
            void printConfiguration(const Configuration *) const;

        protected:
            virtual double
            distance(const Configuration *a, const Configuration *b) const;

            virtual Vertex addConfiguration(Configuration *q);
            void addEdge(const Vertex a, const Vertex b);

            ob::Cost costHeuristic(Vertex u, Vertex v) const;

            /// Shortest path on quotient-graph
            ob::PathPtr getPath(const Vertex &start, const Vertex &goal);

            /// Nearest neighbor structure for quotient space configurations
            RoadmapNeighborsPtr nearestDatastructure_;
            Graph graph_;
            ob::PathPtr solutionPath_;
            RNG rng_;
            typedef boost::minstd_rand RNGType;
            RNGType rng_boost;

            /// \brief Length of graph (useful for determing importance of 
            /// quotient-space
            double graphLength_{0.0};
        };
    }
}

#endif
