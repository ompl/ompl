#ifndef OMPL_MULTILEVEL_PLANNERS_EXPLORER_LOCALMINIMATREE_
#define OMPL_MULTILEVEL_PLANNERS_EXPLORER_LOCALMINIMATREE_

#include <ompl/util/ClassForward.h>
#include <ompl/base/State.h>
#include <ompl/base/Cost.h>
#include <ompl/multilevel/datastructures/BundleSpaceGraph.h>
#include <vector>
#include <mutex>

namespace ompl
{
    namespace base
    {
        OMPL_CLASS_FORWARD(SpaceInformation);
        OMPL_CLASS_FORWARD(Path);
    }
    namespace multilevel
    {
        using VertexPath = std::vector<BundleSpaceGraph::Vertex>;
        using StatesPath = std::vector<base::State*>;

        class LocalMinimaNode
        {
          public:
            LocalMinimaNode(base::SpaceInformationPtr si, base::PathPtr path);
            LocalMinimaNode(base::SpaceInformationPtr si, StatesPath& states);
            LocalMinimaNode(base::SpaceInformationPtr si, VertexPath& vertices);
            double getCost()
            {
              return cost_.value();
            }
            void setVertexPath(VertexPath& vertices)
            {
                vpath_ = vertices;
                hasVertexRepresentation = true;
                customRepresentation = nullptr;
            }
            void setPathPtr(base::PathPtr path)
            {
                path_ = path;
                hasPathPtrRepresentation = true;
                customRepresentation = nullptr;
            }
            const StatesPath& asStates() const;
            const VertexPath& asVertices() const;
            const base::PathPtr& asPathPtr() const;
            void setCost(double c)
            {
              cost_ = base::Cost(c);
            }
            int getLevel() const
            {
              return level_;
            }
            void setLevel(int level)
            {
              level_ = level;
            }

            void *customRepresentation{nullptr};

          private:
            //content
            base::Cost cost_;

            int level_;

            base::SpaceInformationPtr si_;

            bool hasStatesRepresentation{false};
            bool hasVertexRepresentation{false};
            bool hasPathPtrRepresentation{false};

            //different internal representations of path (on-demand)
            VertexPath vpath_;
            StatesPath spath_;
            base::PathPtr path_;
            
            LocalMinimaNode *parent_{nullptr};
        };

        //The PlannerData class for Local minima tree
        class LocalMinimaTree
        {
          public:
            LocalMinimaTree() = delete;
            LocalMinimaTree(const LocalMinimaTree &) = delete;
            LocalMinimaTree(std::vector<base::SpaceInformationPtr>);
            ~LocalMinimaTree(); 

            unsigned int getNumberOfMinima(unsigned int level) const;
            unsigned int getNumberOfMinima() const;
            unsigned int getNumberOfLevel() const;
            unsigned int getNumberOfLevelContainingMinima() const;

            std::vector<int> getSelectedMinimum() const;
            void setSelectedMinimum(std::vector<int> index);

            const std::vector<base::State*>& getSelectedMinimumAsStateVector(int level) const;

            /* Browser-like functionalities to select minimum (can be
             * mapped to hjkl, arrow keys or mouse buttons) */
            void setSelectedMinimumPrev();
            void setSelectedMinimumNext();
            void setSelectedMinimumCollapse();
            void setSelectedMinimumExpand();
            void printSelectedMinimum();

            LocalMinimaNode* getPath(int level, int index) const;
            LocalMinimaNode* getSelectedPath() const;
            double getPathCost(int level, int index) const;

            void sanityCheckLevelIndex(int level, int index) const;


            // void updatePath(int level, int index, VertexPath path, double cost);
            // void addPath(int level, VertexPath path, double cost);

            LocalMinimaNode* addPath(base::PathPtr path, double cost, int level);
            LocalMinimaNode* updatePath(base::PathPtr path, double cost, int level, int index);

            std::mutex& getLock();

          protected:
            std::mutex lock_;

            //needed to convert between representations of tree
            std::vector<base::SpaceInformationPtr> siVec_; 
            std::vector<int> selectedMinimum_;
            std::vector<std::vector<LocalMinimaNode*>> tree_;

            int numberOfMinima_{0};
            int levels_{0};

        };

    }
}
#endif
