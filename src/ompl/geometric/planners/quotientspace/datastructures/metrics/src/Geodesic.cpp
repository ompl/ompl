#include <ompl/geometric/planners/quotientspace/datastructures/metrics/Geodesic.h>
using namespace ompl::geometric;
using Configuration = ompl::geometric::BundleSpaceGraph::Configuration;

BundleSpaceMetricGeodesic::BundleSpaceMetricGeodesic(
    BundleSpaceGraph* bundleSpaceGraph):
  BaseT(bundleSpaceGraph)
{
}

double BundleSpaceMetricGeodesic::distanceBundle(
    const Configuration *xStart, 
    const Configuration *xDest)
{
    return bundleSpaceGraph_->getBundle()->distance(xStart->state, xDest->state);
}

double BundleSpaceMetricGeodesic::distanceFiber(
    const Configuration *xStart, 
    const Configuration *xDest)
{
		if(bundleSpaceGraph_->getFiberDimension() > 0)
		{
        bundleSpaceGraph_->projectFiber(xStart->state, xFiberStartTmp_);
        bundleSpaceGraph_->projectFiber(xDest->state, xFiberDestTmp_);
        double d = bundleSpaceGraph_->getFiber()->distance(
            xFiberStartTmp_, xFiberDestTmp_);
        return d;
		}else{
        return 0.0;
		}
}

double BundleSpaceMetricGeodesic::distanceBase(
    const Configuration *xStart, 
    const Configuration *xDest)
{
		if(bundleSpaceGraph_->getBaseDimension() > 0)
		{
        bundleSpaceGraph_->projectBase(xStart->state, xBaseStartTmp_);
        bundleSpaceGraph_->projectBase(xDest->state, xBaseDestTmp_);
        double d = bundleSpaceGraph_->getBase()->distance(
            xBaseStartTmp_, xBaseDestTmp_);
        return d;
		}else{
        return 0.0;
		}
}

void BundleSpaceMetricGeodesic::interpolateBundle(
    const Configuration *q_from, 
    const Configuration *q_to, 
    const double step, 
    Configuration* q_interp)
{
    bundleSpaceGraph_->getBundle()->getStateSpace()->interpolate(
        q_from->state, q_to->state, step, q_interp->state);
}

// void interpolateBundle(
//     const Configuration *xStart, 
//     const Configuration *xDest) override;
// void interpolateFiber(
//     const Configuration *xStart, 
//     const Configuration *xDest) override;
// void interpolateBase(
//     const Configuration *xStart, 
//     const Configuration *xDest) override;

////############################################################################
////Distance Functions
////############################################################################
//double QuotientMetric::DistanceQ1(const og::QuotientCover::Configuration *q_from, const og::QuotientCover::Configuration *q_to)
//{
//  return quotient_cover->GetQ1()->distance(q_from->state, q_to->state);
//}
//double QuotientMetric::DistanceX1(const QuotientCover::Configuration *q_from, const QuotientCover::Configuration *q_to)
//{
//  if(quotient_cover->GetX1Dimension()>0)
//  {
//    ob::State *stateFrom = quotient_cover->GetX1()->allocState();
//    ob::State *stateTo = quotient_cover->GetX1()->allocState();
//    quotient_cover->ProjectX1Subspace(q_from->state, stateFrom);
//    quotient_cover->ProjectX1Subspace(q_to->state, stateTo);
//    double d = quotient_cover->GetX1()->distance(stateFrom, stateTo);
//    quotient_cover->GetX1()->freeState(stateFrom);
//    quotient_cover->GetX1()->freeState(stateTo);
//    return d;
//  }else{
//    return 0;
//  }
//}

//double QuotientMetric::DistanceConfigurationNeighborhood(const Configuration *q_from, const Configuration *q_to)
//{
//  double d_to = q_to->GetRadius();
//  double d = DistanceConfigurationConfiguration(q_from, q_to);
//  return std::max(d - d_to, 0.0);
//}

//double QuotientMetric::DistanceConfigurationConfigurationQ1(const Configuration *q_from, const Configuration *q_to)
//{
//  return DistanceQ1(q_from, q_to);
//}
//double QuotientMetric::DistanceNeighborhoodNeighborhoodQ1(const Configuration *q_from, const Configuration *q_to)
//{
//  double d = DistanceQ1(q_from, q_to);
//  double d_from = q_from->GetRadius();
//  double d_to = q_to->GetRadius();
//  double d_open_neighborhood_distance = (double)std::fmax(d - d_from - d_to, 0.0); 
//  return d_open_neighborhood_distance;
//}

//double QuotientMetric::DistanceNeighborhoodNeighborhood(const Configuration *q_from, const Configuration *q_to)
//{
//  double d_from = q_from->GetRadius();
//  double d_to = q_to->GetRadius();
//  double d = DistanceConfigurationConfiguration(q_from, q_to);
//  if(d!=d){
//    std::cout << std::string(80, '-') << std::endl;
//    std::cout << "NaN detected." << std::endl;
//    std::cout << "d_from " << d_from << std::endl;
//    std::cout << "d_to " << d_to << std::endl;
//    std::cout << "d " << d << std::endl;
//    std::cout << "configuration 1: " << std::endl;
//    quotient_cover->Print(q_from, false);
//    std::cout << "configuration 2: " << std::endl;
//    quotient_cover->Print(q_to, false);
//    std::cout << std::string(80, '-') << std::endl;
//    throw "";
//    exit(1);
//  }

//  double d_open_neighborhood_distance = (double)std::fmax(d - d_from - d_to, 0.0); 
//  return d_open_neighborhood_distance;
//}
//double QuotientMetric::DistanceConfigurationConfiguration(const Configuration *q_from, const Configuration *q_to) 
//{
//  return DistanceQ1(q_from, q_to);
//}
////############################################################################
////Interpolate Functions
////############################################################################
//void QuotientMetric::InterpolateQ1(const Configuration *q_from, Configuration *q_to)
//{
//  return InterpolateQ1(q_from, q_to, q_to);
//}
//void QuotientMetric::InterpolateQ1(const Configuration *q_from, const Configuration *q_to, Configuration* q_out)
//{
//  double d = DistanceQ1(q_from, q_to);
//  return InterpolateQ1(q_from, q_to, q_from->GetRadius()/d, q_out);
//}
//void QuotientMetric::InterpolateQ1(const Configuration *q_from, const Configuration *q_to, const double step, Configuration* q_out)
//{
//  return quotient_cover->GetQ1()->getStateSpace()->interpolate(q_from->state, q_to->state, step, q_out->state);
//}

//void QuotientMetric::Interpolate(const Configuration *q_from, Configuration *q_to)
//{
//  return Interpolate(q_from, q_to, q_to);
//}
//void QuotientMetric::Interpolate(const Configuration *q_from, const Configuration *q_to, Configuration *q_interp)
//{
//  double d = DistanceConfigurationConfiguration(q_from, q_to);
//  double radius = q_from->GetRadius();
//  double step_size = radius/d;
//  Interpolate(q_from, q_to, step_size, q_interp);
//}

//void QuotientMetric::Interpolate(const Configuration *q_from, const Configuration *q_to, const double step_size, Configuration *q_interp)
//{
//  return quotient_cover->GetQ1()->getStateSpace()->interpolate(q_from->state, q_to->state, step_size, q_interp->state);
//}


