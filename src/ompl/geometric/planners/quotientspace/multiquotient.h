#pragma once
#include "quotient.h"
#include <type_traits>
#include <queue>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
  namespace geometric
  {
    template <class T, typename Tlast=T>
    class MultiQuotient: public ob::Planner{
        static_assert(std::is_base_of<og::Quotient, T>::value, "Template must inherit from Quotient");
        static_assert(std::is_base_of<og::Quotient, Tlast>::value, "Template must inherit from Quotient");

      public:
        const bool DEBUG{false};
        MultiQuotient(std::vector<ob::SpaceInformationPtr> &si_vec, std::string type = "QuotientPlanner");
        void setProblemDefinition(std::vector<ob::ProblemDefinitionPtr> &pdef_vec_);

        virtual ~MultiQuotient() override;

        void getPlannerData(base::PlannerData &data) const override;
        ob::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
        void setup() override;
        void clear() override;
        void setProblemDefinition(const ob::ProblemDefinitionPtr &pdef) override;

        int GetLevels() const;
        std::vector<int> GetFeasibleNodes() const;
        std::vector<int> GetNodes() const;
        std::vector<int> GetDimensionsPerLevel() const;
        void SetStopLevel(uint level_);

      protected:
        std::vector<base::PathPtr> solutions;
        std::vector<og::Quotient*> quotientSpaces;

        bool foundKLevelSolution{false};
        uint currentQuotientLevel{0};
        uint stopAtLevel;

        std::vector<ob::SpaceInformationPtr> si_vec;
        std::vector<ob::ProblemDefinitionPtr> pdef_vec;

        struct CmpQuotientSpacePtrs
        {
          // ">" operator: smallest value is top in queue
          // "<" operator: largest value is top in queue (default)
          bool operator()(const Quotient* lhs, const Quotient* rhs) const
          {
             return lhs->GetImportance() < rhs->GetImportance();
          }
        };
        typedef std::priority_queue<og::Quotient*, std::vector<og::Quotient*>, CmpQuotientSpacePtrs> QuotientSpacePriorityQueue;
        QuotientSpacePriorityQueue Q;
    };
  };
};
#include "src/multiquotient.ipp"
