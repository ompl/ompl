#ifndef OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_METRICS_BUNDLEMETRIC_
#define OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_METRICS_BUNDLEMETRIC_
#include <ompl/geometric/planners/quotientspace/datastructures/BundleSpaceGraph.h>

namespace ompl
{
  namespace geometric
  {
    OMPL_CLASS_FORWARD(BundleSpaceGraph);
    // class BundleSpaceGraph::Configuration;

    // using BundleSpaceGraphPtr = std::shared_ptr<BundleSpaceGraph>;

    class BundleSpaceMetric
    {
      public:
        // using Configuration = ompl::geometric::BundleSpaceGraph::Configuration;
        using Configuration = BundleSpaceGraph::Configuration;
        // typedef ompl::geometric::BundleSpaceGraph::Configuration Configuration;
        BundleSpaceMetric() = delete;
        BundleSpaceMetric(BundleSpaceGraph*); 

        virtual ~BundleSpaceMetric();

        //############################################################################
        //Distance Functions
        //############################################################################

        virtual double distanceBundle(
            const Configuration *xStart, 
            const Configuration *xDest) = 0;
        virtual double distanceFiber(
            const Configuration *xStart, 
            const Configuration *xDest) = 0;
        virtual double distanceBase(
            const Configuration *xStart, 
            const Configuration *xDest) = 0;

        //############################################################################
        //Interpolate Functions
        //############################################################################

        // virtual void interpolateBundle(
        //     const Configuration *xStart, 
        //     const Configuration *xDest) = 0;
        // virtual void interpolateFiber(
        //     const Configuration *xStart, 
        //     const Configuration *xDest) = 0;
        // virtual void interpolateBase(
        //     const Configuration *xStart, 
        //     const Configuration *xDest) = 0;

        // virtual void Interpolate(const Configuration *q_from, const Configuration *q_to, const double step, Configuration* q_interp);
        // virtual void Interpolate(const Configuration *q_from, const Configuration *q_to, Configuration* q_interp);
        // void Interpolate(const Configuration *q_from, Configuration *q_to);

        // void InterpolateQ1(const Configuration *q_from, Configuration *q_to);
        // void InterpolateQ1(const Configuration *q_from, const Configuration *q_to, Configuration* q_out);
        // void InterpolateQ1(const Configuration *q_from, const Configuration *q_to, const double step, Configuration* q_interp);

      protected:

        BundleSpaceGraph* bundleSpaceGraph_;

        base::State *xFiberStartTmp_;
        base::State *xFiberDestTmp_;
        base::State *xBaseStartTmp_;
        base::State *xBaseDestTmp_;
    };
  }
}


#endif
