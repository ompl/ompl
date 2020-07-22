#pragma once
#include <ompl/multilevel/datastructures/BundleSpaceGraph.h>
#include <ompl/multilevel/datastructures/BundleSpaceGraphSparse.h>

#include <ompl/multilevel/datastructures/BundleSpaceSequence.h>
#include <ompl/multilevel/planners/explorer/datastructures/PathSpace.h>
#include <type_traits>
#include <queue>

namespace ob = ompl::base;
namespace og = ompl::multilevel;

namespace ompl
{
    namespace multilevel
    {

        template <class T>
        class MultiLevelPathSpace : public og::BundleSpaceSequence<T>
        {
            static_assert(std::is_base_of<og::BundleSpace, T>::value, 
                "Template must inherit from BundleSpace");
            static_assert(std::is_base_of<og::PathSpace, T>::value, 
                "Template must inherit from PathSpace");

            typedef og::BundleSpaceSequence<T> BaseT;
        public:
            const bool DEBUG{false};

            MultiLevelPathSpace(std::vector<ob::SpaceInformationPtr> &siVec, 
                std::string type = "MultiLevelPathSpace");
            virtual ~MultiLevelPathSpace() override;

            void getPlannerData(ob::PlannerData &data) const override;

            ob::PlannerStatus solve(const ob::PlannerTerminationCondition &ptc) override;
            void setup() override;
            void clear() override;
            void setLocalMinimumSelection( std::vector<int> selection);

        protected:
            double pathBias{0.8}; //[0,1]

            T *root{nullptr};
            T *current{nullptr};
            std::vector<int> selectedLocalMinimum_;
        };
    }
}

#include "MultiLevelPathSpaceImpl.h"
// #endif
