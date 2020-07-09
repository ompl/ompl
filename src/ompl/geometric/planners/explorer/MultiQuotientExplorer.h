// #ifndef OMPL_GEOMETRIC_PLANNERS_EXPLORER_MOTIONEXPLORERIMPL_
// #define OMPL_GEOMETRIC_PLANNERS_EXPLORER_MOTIONEXPLORERIMPL_
#pragma once
#include <ompl/geometric/planners/multilevel/datastructures/BundleSpaceGraph.h>
#include <ompl/geometric/planners/multilevel/datastructures/BundleSpaceGraphSparse.h>

#include <ompl/geometric/planners/multilevel/datastructures/BundleSpaceSequence.h>
#include <type_traits>
#include <queue>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
    namespace geometric
    {

        template <class T>
        class MotionExplorerImpl : public og::BundleSpaceSequence<T>
        {
            static_assert(std::is_base_of<og::BundleSpaceGraph, T>::value,
                "Template must inherit from BundleSpaceGraphSparse");

            typedef og::BundleSpaceSequence<T> BaseT;
        public:
            const bool DEBUG{false};

            MotionExplorerImpl(std::vector<ob::SpaceInformationPtr> &siVec, 
                std::string type = "MotionExplorer");
            virtual ~MotionExplorerImpl() override;

            void getPlannerData(ob::PlannerData &data) const override;

            //void getLocalMinimaTree(ob::LocalMinimaTree &data) const override;

            ob::PlannerStatus solve(const ob::PlannerTerminationCondition &ptc) override;
            void setup() override;
            void clear() override;
            void setLocalMinimumSelection( std::vector<int> selection);

        protected:
            double pathBias{0.8}; //[0,1]

            og::BundleSpaceGraph *root{nullptr};
            og::BundleSpaceGraph *current{nullptr};
            std::vector<int> selectedLocalMinimum_;
        };
    }
}

#include "MultiQuotientExplorerImpl.h"
// #endif
