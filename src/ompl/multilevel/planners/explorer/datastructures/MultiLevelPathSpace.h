#ifndef OMPL_MULTILEVEL_PLANNERS_EXPLORER_MULTILEVELMOTIONEXPLORER_
#define OMPL_MULTILEVEL_PLANNERS_EXPLORER_MULTILEVELMOTIONEXPLORER_
#include <ompl/multilevel/datastructures/BundleSpaceGraph.h>
#include <ompl/multilevel/datastructures/BundleSpaceGraphSparse.h>

#include <ompl/multilevel/datastructures/BundleSpaceSequence.h>
#include <ompl/multilevel/planners/explorer/datastructures/PathSpace.h>
#include <type_traits>
#include <queue>

namespace ompl
{
    namespace multilevel
    {
        OMPL_CLASS_FORWARD(LocalMinimaTree);

        template <class T>
        class MultiLevelPathSpace : public BundleSpaceSequence<T>
        {
            static_assert(std::is_base_of<BundleSpace, T>::value, "Template must inherit from BundleSpace");

            using BaseT = BundleSpaceSequence<T>;

        public:
            const bool DEBUG{false};

            MultiLevelPathSpace(std::vector<base::SpaceInformationPtr> &siVec,
                                std::string type = "MultiLevelPathSpace");
            virtual ~MultiLevelPathSpace() override;

            void getPlannerData(base::PlannerData &data) const override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
            void setup() override;
            void clear() override;

            LocalMinimaTreePtr &getLocalMinimaTree();

        protected:
            double pathBias{0.8};  //[0,1]

            T *current{nullptr};
            // std::vector<int> selectedLocalMinimum_;

            LocalMinimaTreePtr localMinimaTree_;

            enum ExtensionMode
            {
                AUTOMATIC_UNIFORM = 0,
                AUTOMATIC_FAST_DOWNWARD = 1,
                MANUAL = 2
            };

            ExtensionMode mode;
        };
    }
}

#include "MultiLevelPathSpaceImpl.h"
#endif
