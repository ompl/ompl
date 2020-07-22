#pragma once
#include <ompl/multilevel/datastructures/BundleSpaceGraph.h>
#include <ompl/base/PlannerData.h>

namespace ompl{

    namespace multilevel{

        class PathVisibilityChecker;
        class PathSpace{

            using VertexPath = std::vector<BundleSpaceGraph::Vertex>;

          public:

            PathSpace() = delete;
            PathSpace(BundleSpaceGraph*);
            ~PathSpace();

            void setSelectedPath(unsigned int);

            unsigned int getSelectedPath();

            virtual unsigned int getNumberOfPaths() const;

            void updatePath(unsigned int k, VertexPath p, double cost);

            void addPath(VertexPath p, double cost);

            double getPathCost(unsigned int k) const;

            std::vector<BundleSpaceGraph::Vertex>& getCriticalPath(unsigned int k);

            void getPlannerData(base::PlannerData &data, BundleSpaceGraph* bundleGraph) const;

            PathVisibilityChecker *getPathVisibilityChecker();

            void getPathIndices(
                const std::vector<BundleSpaceGraph::Vertex> &vertices, 
                std::vector<int> &idxPath) const;

          protected:
            BundleSpaceGraph *bundleSpaceGraph_;

            PathVisibilityChecker *pathVisibilityChecker_{nullptr};

            int selectedPath_{-1};

            std::vector<VertexPath> criticalPaths_;

            std::vector<double> criticalPathsCost_;
        };
    }
}
