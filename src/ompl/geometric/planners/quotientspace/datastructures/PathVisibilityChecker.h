#pragma once
#include <ompl/geometric/planners/quotientspace/datastructures/QuotientSpaceGraph.h>
#include <ompl/base/State.h>
#include <ompl/geometric/SimpleSetup.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
  namespace geometric
  {
    class PathVisibilityChecker
    {

    public:

      PathVisibilityChecker(const ob::SpaceInformationPtr &si);
      ~PathVisibilityChecker(void);

      bool IsPathVisible(std::vector<QuotientSpaceGraph::Vertex> &v1, std::vector<QuotientSpaceGraph::Vertex> &v2, QuotientSpaceGraph::Graph &graph);
      bool IsPathVisible(std::vector<ob::State*> &s1, std::vector<ob::State*> &s2);
      bool IsPathVisibleSO2(std::vector<ob::State*> &s1, std::vector<ob::State*> &s2);
      bool isPathClockwise(std::vector<ob::State*> &spath);
      bool CheckValidity(const std::vector<ob::State*> &s);

      void Test1();
      void Test2();
      void Test3(int F=0);

    protected:
      bool isDynamic{false};
      ob::SpaceInformationPtr si_;
      ob::SpaceInformationPtr si_local;
      ob::State *lastValidState;

      ob::StateSpacePtr R2space_;
      ob::RealVectorStateSpace *R2;
      SimpleSetupPtr ss;

      ob::ScopedStatePtr start;
      ob::ScopedStatePtr goal;

      float max_planning_time_path_path{0.2};
      float epsilon_goalregion{0.05};

    private:
      std::vector<ob::State*> StatesFromVector( 
          const std::vector<double> &sx, 
          const std::vector<double> &sy);
      std::vector<ob::State*> StatesFromVectorSO2R1( 
          const std::vector<double> &st, 
          const std::vector<double> &sx);
      std::vector<ob::State*> StatesFromVector( 
          const std::vector<double> &sx, 
          const std::vector<double> &sy, 
          const std::vector<double> &st);

    };
  }
}
