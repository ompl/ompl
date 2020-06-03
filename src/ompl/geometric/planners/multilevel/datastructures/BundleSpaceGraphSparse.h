#ifndef OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_BUNDLEGRAPH_SPARSE_
#define OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_BUNDLEGRAPH_SPARSE_

#include <ompl/geometric/planners/multilevel/datastructures/BundleSpaceGraph.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/geometric/PathGeometric.h>
#include "ompl/geometric/PathSimplifier.h"

#include <boost/graph/subgraph.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/random/linear_congruential.hpp>
#include <boost/random/variate_generator.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
    namespace geometric
    {
        class BundleSpaceGraphSparse : public ompl::geometric::BundleSpaceGraph
        {
            using BaseT = ompl::geometric::BundleSpaceGraph;

        public:
            BundleSpaceGraphSparse(const ob::SpaceInformationPtr &si, BundleSpace *parent = nullptr);
            virtual ~BundleSpaceGraphSparse() override;

            virtual void grow() = 0;
            virtual bool getSolution(ompl::base::PathPtr &solution) override;

            virtual void getPlannerData(ob::PlannerData &data) const override;
            void getPlannerDataRoadmap(ob::PlannerData &data, std::vector<int> pathIdx) const;

            virtual Vertex addConfiguration(Configuration *q) override;
            virtual void deleteConfiguration(Configuration *q) override;
            Vertex addConfigurationSparse(Configuration *q);
            void addEdgeSparse(const Vertex a, const Vertex b);

            virtual void setup() override;
            virtual void clear() override;

            virtual void init() override;

            // Using same conditions as SPARS algorithm to determine sparse graph
            // addition
            void findGraphNeighbors(Configuration *q, std::vector<Configuration *> &graphNeighborhood,
                                    std::vector<Configuration *> &visibleNeighborhood);
            bool checkAddCoverage(Configuration *q, std::vector<Configuration *> &visibleNeighborhoods);
            bool checkAddConnectivity(Configuration *q, std::vector<Configuration *> &visibleNeighborhood);
            bool checkAddInterface(Configuration *q, std::vector<Configuration *> &graphNeighborhood,
                                   std::vector<Configuration *> &visibleNeighborhood);
            bool checkAddPath(Configuration *q);

            void updateRepresentatives(Configuration *q);
            void getInterfaceNeighborRepresentatives(Configuration *q, std::set<Vertex> &interfaceRepresentatives);
            void addToRepresentatives(Vertex q, Vertex rep, const std::set<Vertex> &interfaceRepresentatives);
            void removeFromRepresentatives(Configuration *q);

            void getInterfaceNeighborhood(Configuration *q, std::vector<Vertex> &interfaceNeighborhood);
            void computeVPP(Vertex v, Vertex vp, std::vector<Vertex> &VPPs);
            void computeX(Vertex v, Vertex vp, Vertex vpp, std::vector<Vertex> &Xs);
            Vertex getInterfaceNeighbor(Vertex q, Vertex rep);
            void computeDensePath(const Vertex &start, const Vertex &goal, std::deque<base::State *> &path);
            bool addPathToSpanner(const std::deque<base::State *> &dense_path, Vertex vp, Vertex vpp);

            void updatePairPoints(Configuration *q);

            virtual void print(std::ostream &out) const override;
            bool hasSparseGraphChanged();

            virtual const Configuration *nearest(const Configuration *s) const;

        protected:
            double sparseDelta_{0.};
            double denseDelta_{0.};  // delta for dense graph -> move this BundleSpaceGraph.h
            double sparseDeltaFraction_{0.15};
            double denseDeltaFraction_{0.05};
            unsigned Nold_v{0};
            unsigned Nold_e{0};

            void setSparseDeltaFraction(double D)
            {
                sparseDeltaFraction_ = D;
            }
            double getSparseDeltaFraction() const
            {
                return sparseDeltaFraction_;
            }

            virtual void uniteComponentsSparse(Vertex m1, Vertex m2);
            bool sameComponentSparse(Vertex m1, Vertex m2);

            std::map<Vertex, VertexRank> vrankSparse;
            std::map<Vertex, Vertex> vparentSparse;
            boost::disjoint_sets<boost::associative_property_map<std::map<Vertex, VertexRank>>,
                                 boost::associative_property_map<std::map<Vertex, Vertex>>>
                disjointSetsSparse_{boost::make_assoc_property_map(vrankSparse),
                                    boost::make_assoc_property_map(vparentSparse)};

            void clearDynamic();
            ompl::base::PathPtr getPathSparse(const Vertex &start, const Vertex &goal);
            ompl::base::Cost costHeuristicSparse(Vertex u, Vertex v) const;

            Graph graphSparse_;
            RoadmapNeighborsPtr nearestSparse_;
            std::vector<Configuration *> graphNeighborhood;
            std::vector<Configuration *> visibleNeighborhood;

            Vertex v_start_sparse;
            Vertex v_goal_sparse;

            // From SPARS
            /** \brief A counter for the number of consecutive failed iterations of the algorithm */
            unsigned int consecutiveFailures_{0u};

            /** \brief The stretch factor in terms of graph spanners for SPARS to check against */
            double stretchFactor_{3.};

            /** \brief Geometric Path variable used for smoothing out paths. */
            PathGeometric geomPath_;

            /** \brief A path simplifier used to simplify dense paths added to S */
            PathSimplifierPtr psimp_;

            std::vector<Vertex> startGoalVertexPath_;
            std::vector<double> lengthsStartGoalVertexPath_;
        };
    };
};

#endif
