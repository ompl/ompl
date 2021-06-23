#pragma once
#include <ompl/multilevel/datastructures/BundleSpaceGraph.h>
#include <queue>

namespace ompl
{
    namespace multilevel
    {
        /* A sparse tree which is also a nearest neighbor structure and a priority queue.*/
        using Configuration = ompl::multilevel::BundleSpaceGraph::Configuration;
        enum ExtensionReturnType
        {
            EXTENSION_SUCCESS = 0,
            EXTENSION_FAILURE_INVALID_STATE = 1,
            EXTENSION_FAILURE_INSIDE_COVER = 2,
            EXTENSION_FAILURE_NO_CONNECTION = 2
        };
        struct ConfigurationAnalytics
        {
            Configuration *config;
            int numberOfExtensions{0};
            int numberOfSuccessfulExtensions{0};
            int numberOfUnsuccessfulSubsequentExtensions{0};
            bool isConverged{false};
        };

        class SparseTree
        {
          public:
            using TreeData = std::shared_ptr<NearestNeighbors<Configuration *>>;
            SparseTree() = delete;
            SparseTree(TreeData& data);
            void clear();
            void setup();

            bool isConverged(); //Check if all nodes have converged

            std::size_t size(); //Number of nodes in tree

            void add(Configuration *x);
            Configuration* nearest(Configuration *x);

            //Priority queue specific functionality
            void push(Configuration *x, ExtensionReturnType type);
            Configuration* pop();
            ConfigurationAnalytics* getAnalyticsFromConfiguration(Configuration *x);
          private:
            TreeData data_;

            std::vector<ConfigurationAnalytics*> analytics_;
            std::unordered_map<int, int> indexConfigToAnalytics_;

            struct CmpTreeElements
            {
                // ">" operator: smallest value is top in queue
                // "<" operator: largest value is top in queue (default)
                bool operator()(const Configuration *lhs, const Configuration *rhs) const
                {
                    return lhs->importance < rhs->importance;
                }
            };
            typedef std::priority_queue<Configuration *, std::vector<Configuration *>, CmpTreeElements>
                TreeElementPriorityQueue;
            TreeElementPriorityQueue treeElementsPriorityQueue_;
        };
        using SparseTreePtr = std::shared_ptr<SparseTree>;
    }
}
