#ifndef OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_BUNDLEGRAPH_SPARSE_
#define OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_BUNDLEGRAPH_SPARSE_

#include "PathVisibilityChecker.h"
#include <ompl/geometric/planners/quotientspace/datastructures/BundleSpaceGraph.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/geometric/PathGeometric.h>
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
    class BundleSpaceGraphSparse: public ompl::geometric::BundleSpaceGraph{

        using BaseT = ompl::geometric::BundleSpaceGraph;
      public:

        BundleSpaceGraphSparse(const ob::SpaceInformationPtr &si, BundleSpace *parent = nullptr);
        virtual ~BundleSpaceGraphSparse() override;

        virtual void grow() = 0;
        virtual void getPlannerData(ob::PlannerData &data) const override;
        void getPlannerDataRoadmap(ob::PlannerData &data, std::vector<int> pathIdx) const;

        virtual void deleteConfiguration(Configuration *q);
        virtual Vertex addConfiguration(Configuration *q) override;
        Vertex addConfigurationSparse(Configuration *q);
        void addEdgeSparse(const Vertex a, const Vertex b);

        // void AddEdge(const Configuration* q1, const Configuration* q2);
        virtual void setup() override;
        virtual void clear() override;
        void TestVisibilityChecker();

        virtual void Init();
        //Copied from SPARS
        void findGraphNeighbors(Configuration *q, std::vector<Configuration*> &graphNeighborhood,
                                std::vector<Configuration*> &visibleNeighborhood);
        bool checkAddConnectivity(Configuration* q, std::vector<Configuration*> &visibleNeighborhood);
        bool checkAddInterface(Configuration *q, std::vector<Configuration*> &graphNeighborhood, std::vector<Configuration*> &visibleNeighborhood);

        void Rewire(Vertex &v);
        void Rewire();
        void printAllPathsUtil(Vertex u, Vertex d, bool visited[], int path[], int &path_index);
        virtual void enumerateAllPaths();
        void removeReducibleLoops();
        void removeEdgeIfReductionLoop(const Edge &e);
        unsigned int getNumberOfPaths() const;
        const std::vector<ob::State*> getKthPath(uint k) const;
        void getPathIndices(const std::vector<ob::State*> &states, std::vector<int> &idxPath) const;
        bool isProjectable(const std::vector<ob::State*> &pathBundle) const;
        int getProjectionIndex(const std::vector<ob::State*> &pathBundle) const;

        int selectedPath{-1}; //selected path to sample from (if children try to sample this space)
        bool sampleFromDatastructure(ob::State *q_random_graph) override;

        PathVisibilityChecker* getPathVisibilityChecker();

        void pushPathToStack(std::vector<ob::State*> &path);
        void removeLastPathFromStack();

        virtual void print(std::ostream &out) const override;
        bool hasSparseGraphChanged();

        virtual const Configuration *nearest(const Configuration *s) const;
        void freePath(std::vector<ob::State*> path, const ob::SpaceInformationPtr &si) const;
        std::vector<ob::State*> getProjectedPath(const std::vector<ob::State*> pathBundle, const ob::SpaceInformationPtr &si) const;
    protected:

        double sparseDelta_{0.};
        double sparseDeltaFraction_{0.25};
        double pathBias_{0.};
        double pathBiasFraction_{0.05};
        double kPRMStarConstant_;
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
        uint numberVertices{0};

        unsigned numberOfFailedAddingPathCalls{0};
        unsigned Nhead{5}; //head -nX (to display only X top paths)
        std::vector<og::PathGeometric> pathStack_;
        std::vector<std::vector<ob::State*>> pathStackHead_;
        void PrintPathStack();

        std::vector<int> GetSelectedPathIndex() const;

        virtual void uniteComponentsSparse(Vertex m1, Vertex m2);
        bool sameComponentSparse(Vertex m1, Vertex m2);
        // boost::disjoint_sets<boost::associative_property_map<std::map<Vertex, VertexRank> >, boost::associative_property_map<std::map<Vertex, Vertex> > > 
        //   disjointSetsSparse_{boost::make_assoc_property_map(vrank), boost::make_assoc_property_map(vparent)};
        std::map<Vertex, VertexRank> vrankSparse;
        std::map<Vertex, Vertex> vparentSparse;
        boost::disjoint_sets<boost::associative_property_map<std::map<Vertex, VertexRank> >, boost::associative_property_map<std::map<Vertex, Vertex> > > 
          disjointSetsSparse_{boost::make_assoc_property_map(vrankSparse), boost::make_assoc_property_map(vparentSparse)};


        void clearDynamic();
        ompl::base::PathPtr getPathSparse(const Vertex &start, const Vertex &goal);
        ompl::base::Cost costHeuristicSparse(Vertex u, Vertex v) const;
        Graph graphSparse_;
        RoadmapNeighborsPtr nearestSparse_;
        std::vector<Configuration*> graphNeighborhood;
        std::vector<Configuration*> visibleNeighborhood;

        Vertex v_start_sparse;
        Vertex v_goal_sparse;

        PathVisibilityChecker* pathVisibilityChecker_{nullptr};

    };
  };
};

#endif
