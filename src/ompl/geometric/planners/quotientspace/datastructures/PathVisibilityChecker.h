#pragma once
#include <ompl/geometric/planners/quotientspace/datastructures/QuotientSpaceGraph.h>
#include <ompl/base/State.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/control/Control.h>
#include <ompl/control/DirectedControlSampler.h>

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
      bool IsPathDynamicallyVisible(std::vector<ob::State*> &s1, std::vector<ob::State*> &s2, std::vector<ob::State*> &sLocal);
      bool IsPathVisibleSO2(std::vector<ob::State*> &s1, std::vector<ob::State*> &s2);
      bool isPathClockwise(std::vector<ob::State*> &spath);
      bool CheckValidity(const std::vector<ob::State*> &s);

      void createStateAt(ob::SpaceInformationPtr si_,const std::vector<ob::State*> &path, const double &pathLength, const std::vector<double> &distances, const double newPosition, ob::State* s_interpolate) const;
      void computePathLength(ob::SpaceInformationPtr si_,const std::vector<ob::State*> &path, std::vector<double> &stateDistances, double &pathLength);
      //bool isStepDynamicallyFeasible(const ob::State* s_start, ob::State* s_target, const ompl::control::Control* c_previous, ompl::control::Control* c_current, const double targetRegion, const ompl::control::SpaceInformation* siC, const ompl::control::DirectedControlSamplerPtr sampler);
      bool isPathDynamicallyFeasible(const std::vector<ompl::base::State*> path) const;

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
      
      bool stepFeasible;
      std::vector<ob::State*> statesDyn;
      std::vector<ob::State*> statesDyn_next;
      //std::vector<ompl::control::Control*> controls;
      //std::vector<ompl::control::Control*> controls_next;
      int controlSamples{20};
      double pathSamples{10.};
      ompl::control::DirectedControlSamplerPtr sDCSampler;
      //double distanceThreshold{0.1};
      
//      ompl::control::DirectedControlSamplerPtr simpleDirectedControlSamplerAllocator(const ompl::control::SpaceInformationPtr siC) const;


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
