#ifndef OMPL_GEOMETRIC_PLANNERS_QUOTIENTSPACE_MotionExplorerImpl_
#define OMPL_GEOMETRIC_PLANNERS_QUOTIENTSPACE_MotionExplorerImpl_
#include <ompl/geometric/planners/quotientspace/algorithms/MultiQuotient.h>
#include <ompl/geometric/planners/quotientspace/datastructures/BundleSpaceGraphSparse.h>
#include <type_traits>
#include <queue>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
    namespace geometric
    {

        template <class T>
        class MotionExplorerImpl : public og::MultiQuotient<T>
        {
            static_assert(std::is_base_of<og::BundleSpaceGraphSparse, T>::value, 
                "Template must inherit from BundleSpaceGraphSparse");

            typedef og::MultiQuotient<T> BaseT;
        public:
            const bool DEBUG{false};

            MotionExplorerImpl(std::vector<ob::SpaceInformationPtr> &siVec, 
                std::string type = "MotionExplorer");
            virtual ~MotionExplorerImpl() override;

            void getPlannerData(ob::PlannerData &data) const override;
            ob::PlannerStatus solve(const ob::PlannerTerminationCondition &ptc) override;
            void setup() override;
            void clear() override;
            void setSelectedPath( std::vector<int> selectedPath);

        protected:
            double pathBias{0.8}; //[0,1]

            og::BundleSpaceGraphSparse *root{nullptr};
            og::BundleSpaceGraphSparse *current{nullptr};
            std::vector<int> selectedPath_;
        };
    }
}
#include "MultiQuotientExplorerImpl.h"
#endif


