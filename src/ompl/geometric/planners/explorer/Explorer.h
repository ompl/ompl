#ifndef OMPL_GEOMETRIC_PLANNERS_QUOTIENTSPACE_MotionExplorer_
#define OMPL_GEOMETRIC_PLANNERS_QUOTIENTSPACE_MotionExplorer_
#include "QuotientGraphSparse.h"
#include <ompl/geometric/planners/quotientspace/algorithms/MultiQuotient.h>
#include <type_traits>
#include <queue>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
    namespace geometric
    {
        template <class T>
        class MotionExplorer : public og::MultiQuotient<T>
        {
            static_assert(std::is_base_of<og::QuotientGraphSparse, T>::value, "Template must inherit from QuotientGraphSparse");

            typedef og::MultiQuotient<T> BaseT;
        public:
            const bool DEBUG{false};

            MotionExplorer(std::vector<ob::SpaceInformationPtr> &siVec, 
                std::string type = "MotionExplorer");
            virtual ~MotionExplorer() override;

            void getPlannerData(ob::PlannerData &data) const override;
            ob::PlannerStatus solve(const ob::PlannerTerminationCondition &ptc) override;
            void setup() override;
            void clear() override;
            void setSelectedPath( std::vector<int> selectedPath);

        protected:
            double pathBias{0.8}; //[0,1]

            og::QuotientGraphSparse *root{nullptr};
            og::QuotientGraphSparse *current{nullptr};
            std::vector<int> selectedPath_;
        };
    }
}
#include "Explorer.ipp"
#endif


