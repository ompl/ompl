#pragma once
#include <ompl/geometric/planners/quotientspace/datastructures/BundleSpaceGraphSparse.h>
#include <ompl/datastructures/PDF.h>
#include <ompl/control/Control.h>
#include <ompl/control/StatePropagator.h>
#include <ompl/control/DirectedControlSampler.h>
#include "PathVisibilityChecker.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
//namespace oc = ompl::control;
//need to set propagationStepSize?

namespace ompl
{
  namespace base
  {
      OMPL_CLASS_FORWARD(OptimizationObjective);
  }
  namespace geometric
  {
    //ExplorerImpl 
    class ExplorerImpl: public og::BundleSpaceGraphSparse{

      using BaseT = ompl::geometric::BundleSpaceGraphSparse;
      public:

        ExplorerImpl(const ob::SpaceInformationPtr &si, BundleSpace *parent_);
        virtual ~ExplorerImpl() override;
        virtual void grow() override;
        virtual void growGeometric();
        virtual void growGeometricExpand();
        virtual void growControl();
        virtual bool getSolution(ob::PathPtr &solution) override;

        virtual void setup() override;
        virtual void clear() override;

        void setGoalBias(double goalBias);
        double getGoalBias() const;
        void setRange(double distance);
        double getRange() const;
        virtual bool hasSolution() override;

        Configuration *q_random{nullptr};
        ompl::control::Control* c_random{nullptr};
        ob::State* s_random;
        ompl::control::StatePropagatorPtr prop;
        ompl::control::DirectedControlSamplerPtr dCSampler;
	
        virtual void getPlannerData(ob::PlannerData &data) const override;

        //############################################################################
        void printAllPathsUtil(Vertex u, Vertex d, bool visited[], int path[], int &path_index);
        virtual void enumerateAllPaths();
        void removeReducibleLoops();
        void removeEdgeIfReductionLoop(const Edge &e);
        unsigned int getNumberOfPaths() const;
        const std::vector<ob::State*> getKthPath(uint k) const;
        void getPathIndices(const std::vector<ob::State*> &states, std::vector<int> &idxPath) const;
        bool isProjectable(const std::vector<ob::State*> &pathBundle) const;
        int getProjectionIndex(const std::vector<ob::State*> &pathBundle) const;
        int selectedPath{-1}; //selected path to sample from (if children try to sample this space)
        virtual void sampleFromDatastructure(ob::State *q_random_graph) override;

        PathVisibilityChecker* getPathVisibilityChecker();
        void pushPathToStack(std::vector<ob::State*> &path);
        void removeLastPathFromStack();
        std::vector<ob::State*> getProjectedPath(const std::vector<ob::State*> pathBundle, const ob::SpaceInformationPtr &si) const;

        void freePath(std::vector<ob::State*> path, const ob::SpaceInformationPtr &si) const;

        PathVisibilityChecker* pathVisibilityChecker_{nullptr};

        unsigned numberOfFailedAddingPathCalls{0};
        unsigned Nhead{5}; //head -nX (to display only X top paths)
        std::vector<og::PathGeometric> pathStack_;
        std::vector<std::vector<ob::State*>> pathStackHead_;
        void PrintPathStack();

        std::vector<int> GetSelectedPathIndex() const;
        //############################################################################
      protected:

        int numberOfControlSamples{10};
        double propStepSize;
        int controlDuration{10};
        double maxDistance{.0};
        double goalBias{.05};
        double epsilon{.0};
        double distanceToGoal{.0};
        double approximateDistanceToGoal{.0};

    };

  };
};
