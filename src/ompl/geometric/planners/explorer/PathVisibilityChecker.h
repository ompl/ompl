#pragma once
#include <ompl/geometric/planners/quotientspace/QuotientSpaceGraph.h>
#include <ompl/base/State.h>

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
      ob::SpaceInformationPtr si_;
      ob::State *lastValidState;

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
