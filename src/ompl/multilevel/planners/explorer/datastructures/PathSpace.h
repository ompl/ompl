#pragma once
#include <ompl/multilevel/datastructures/BundleSpaceGraph.h>
#include <ompl/multilevel/planners/explorer/datastructures/LocalMinimaTree.h>
#include <ompl/base/PlannerData.h>

namespace ompl
{
    namespace base
    {
        OMPL_CLASS_FORWARD(Path);
    }
    namespace multilevel
    {
        OMPL_CLASS_FORWARD(LocalMinimaTree);

        class PathSpace
        {
        public:
            PathSpace() = delete;
            PathSpace(BundleSpaceGraph *);
            ~PathSpace();

            void setLocalMinimaTree(LocalMinimaTreePtr);

            virtual unsigned int getNumberOfPaths() const;

            base::PathPtr VerticesToPathPtr(VertexPath vpath);

            void updatePath(unsigned int k, VertexPath p, double cost);

            void updatePath(unsigned int k, base::PathPtr path, double cost);

            void addPath(VertexPath p, double cost);

            void addPath(base::PathPtr path, double cost);

            double getPathCost(unsigned int k) const;

            void clear();

            const std::vector<BundleSpaceGraph::Vertex> &getMinimumPath(unsigned int k);
            const std::vector<base::State*> &getMinimumPathStates(unsigned int k);

        protected:
            BundleSpaceGraph *bundleSpaceGraph_;

            LocalMinimaTreePtr localMinimaTree_;
        };
    }
}
