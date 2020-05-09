#ifndef OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_METRICS_BUNDLEMETRIC_REACHABILITY_
#define OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_METRICS_BUNDLEMETRIC_REACHABILITY_
#include <ompl/geometric/planners/multilevel/datastructures/metrics/Geodesic.h>

namespace ompl
{
  namespace control
  {
    OMPL_CLASS_FORWARD(SpaceInformation);
    OMPL_CLASS_FORWARD(Control);
    OMPL_CLASS_FORWARD(ControlSpace);
    OMPL_CLASS_FORWARD(RealVectorControlSpace);
  }
  namespace geometric
  {
    OMPL_CLASS_FORWARD(PathGeometric);
    class BundleSpaceMetricReachability: public BundleSpaceMetricGeodesic
    {

//Check out implementations of RG-RRT:
//https://github.com/shloksobti/Reachability-Guided-Rapidly-Exploring-Random-Tree/
//https://github.com/rdspring1/comp450-Reachability-Guided-RRT/
//
// Implementation here just uses the metric aspect of RG-RRT to speed up QRRT

        using BaseT = BundleSpaceMetricGeodesic;

      public:

        BundleSpaceMetricReachability() = delete;
        BundleSpaceMetricReachability(BundleSpaceGraph*); 
        virtual ~BundleSpaceMetricReachability() override;

        virtual double distanceBundle(
            const Configuration *xStart, 
            const Configuration *xDest) override;

        void createReachableSet(Configuration *x);

        void setAverageControl(
            control::ControlSpace *controlSpace, control::Control* ctrl);
        void createControls(
            control::ControlSpace* controlSpace, control::Control *control);

        void checkActiveControlDimensions( 
            ompl::control::ControlSpace *controlSpace);

        void computeControl(int dimensions, std::vector<int> controlIdx);
      private:
        ompl::control::SpaceInformationPtr siC_{nullptr};

        void setControlDimension(
            ompl::control::ControlSpace *controlSpace,
            ompl::control::Control *control,
            int Jdimension,
            bool lowerBound);
        void setControlDimension(
            ompl::control::ControlSpace *controlSpace,
            ompl::control::Control *control,
            int Kcomponent,
            int Jdimension,
            bool lowerBound);
        void createControls(
            ompl::control::ControlSpace *controlSpace,
            ompl::control::Control *control,
            std::vector<int> idx);

        int numberConfigurationsReachableSet_{0};
        int stepSizeReachableSet_{1};

        std::vector<std::pair<int, int>> activeControlDimensions_;

        std::vector<control::Control*> controls_;

        ompl::control::ControlSpacePtr controlSpace_{nullptr};

        double minControlDuration_{0.0};
    };
  }
}

#endif

//Note:
// Implement either by adding Configurations as std::vector<Config> to Config
// (the reachable set)
// During runtime, when adding a config, we also add reachable set by using
// random controls and propagating while valid. Then we add if feasible.
//
// Better: Precompute vector of controls. Then propagate each control for min
// time.
