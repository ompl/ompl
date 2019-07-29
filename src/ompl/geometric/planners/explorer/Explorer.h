#ifndef OMPL_GEOMETRIC_PLANNERS_QUOTIENTSPACE_MotionExplorer_
#define OMPL_GEOMETRIC_PLANNERS_QUOTIENTSPACE_MotionExplorer_
#include "QuotientGraphSparse.h"
#include <ompl/geometric/planners/quotientspace/MultiQuotient.h>
#include <type_traits>
#include <queue>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
    namespace geometric
    {
        //class MotionExplorer : public ob::Planner
        template <class T>
        class MotionExplorer : public og::MultiQuotient<T>
        {
            static_assert(std::is_base_of<og::QuotientGraphSparse, T>::value, "Template must inherit from QuotientGraphSparse");

            typedef og::MultiQuotient<T> BaseT;
        public:
            const bool DEBUG{false};

            MotionExplorer(std::vector<ob::SpaceInformationPtr> &si_vec, 
                std::string type = "MotionExplorer");
            virtual ~MotionExplorer() override;

            void getPlannerData(ob::PlannerData &data) const override;

            //Write different PTC (for reaching topological phase shift,
            //etcetera)
            ob::PlannerStatus solve(const ob::PlannerTerminationCondition &ptc) override;

            void setup() override;
            void clear() override;
            void setSelectedPath( std::vector<int> selectedPath);

        protected:
            double pathBias{0.8}; //[0,1]

            /// Sequence of quotient-spaces
            // std::vector<og::QuotientGraphSparse *> quotientSpaces_;

            og::QuotientGraphSparse *root{nullptr};
            og::QuotientGraphSparse *current{nullptr};
            std::vector<int> selectedPath_;
        };
    }
}
#include "Explorer.ipp"
#endif


