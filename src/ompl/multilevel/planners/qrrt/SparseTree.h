#ifndef OMPL_MULTILEVEL_PLANNERS_SPARSE_TREE_
#define OMPL_MULTILEVEL_PLANNERS_SPARSE_TREE_
#include <ompl/multilevel/datastructures/BundleSpaceGraph.h>
#include <ompl/base/SpaceInformation.h>
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
            EXTENSION_FAILURE_INVALID_STATE,
            EXTENSION_FAILURE_INSIDE_COVER,
            EXTENSION_FAILURE_NO_CONNECTION,
            EXTENSION_FAILURE_SHELL_SAMPLING
        };

        //Essentially a decorator for the Configuration class to track
        //information which can be used for statistical analysis later on
        struct ConfigurationAnalytics
        {
            Configuration *config;
            int numberOfExtensions{0};
            int numberOfSuccessfulExtensions{0};
            int numberOfUnsuccessfulSubsequentExtensions{0};
            bool isConverged{false};
            int extensionInvalidState{0};
            int extensionInsideCover{0};
            int extensionNoConnection{0};
            double radius{0.0};
            void print()
            {
                std::cout << "Config " << config->index 
                  << " radius: " << radius
                  << " (ext: " << numberOfExtensions 
                  << ", unsuccess seq: " << numberOfUnsuccessfulSubsequentExtensions 
                  << "), (extension failures: "
                 << extensionInvalidState << " invalid state, " 
                 << extensionInsideCover << " inside cover, " 
                 << extensionNoConnection << " no connection)" 
                 << std::endl;
            }
        };

        class SparseTree
        {
          public:
            using TreeData = std::shared_ptr<NearestNeighbors<Configuration *>>;
            SparseTree() = delete;
            SparseTree(TreeData& data, base::SpaceInformationPtr si);
            void clear();
            void setup();

            bool isConverged(); //Check if all nodes have converged

            std::size_t size(); //Number of nodes in tree
            std::size_t sizeConvergedNodes(); //Number of nodes converged

            void add(Configuration *x);
            Configuration* nearest(Configuration *x);

            //Priority queue specific functionality
            void push(Configuration *x, ExtensionReturnType type);
            Configuration* pop();
            ConfigurationAnalytics* getAnalytics(Configuration *x);

            double getRadius(Configuration *x);

          private:
            static unsigned int counter_;
            unsigned int id_{0}; //tree identity (if we are using multiple trees)

            double initialRadius_{0.0};
            int maximumExtensions_{1000};

            TreeData data_;
            base::SpaceInformationPtr si_;

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
#endif
