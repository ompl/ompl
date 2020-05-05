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

        void setAverageControl(control::RealVectorControlSpace *controlSpace, control::Control* ctrl);

      private:
        ompl::control::SpaceInformationPtr siC_{nullptr};

        int numberConfigurationsReachableSet_{0};
        std::vector<int> activeControlDimensions_;

        int numberActiveControlDimensions_{0};
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
