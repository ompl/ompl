#include <ompl/geometric/planners/quotientspace/datastructures/metrics/ShortestPath.h>
#include <ompl/geometric/PathGeometric.h>


using namespace ompl::geometric;
using Configuration = ompl::geometric::BundleSpaceGraph::Configuration;

BundleSpaceMetricShortestPath::BundleSpaceMetricShortestPath(
    BundleSpaceGraph* bundleSpaceGraph):
  BaseT(bundleSpaceGraph)
{
}

double BundleSpaceMetricShortestPath::distanceBundle(
    const Configuration *xStart, 
    const Configuration *xDest)
{
    if(bundleSpaceGraph_->getBaseDimension() <= 0){
        return BaseT::distanceBundle(xStart, xDest);
    }else{
        std::vector<const Configuration*> path = getInterpolationPath(xStart, xDest);
        double d = 0;
        for(uint k = 0; k < path.size()-1; k++){
          d += BaseT::distanceBundle(path.at(k), path.at(k+1));
        }
        return d;
    }
}

double BundleSpaceMetricShortestPath::distanceFiber(
    const Configuration *xStart, 
    const Configuration *xDest)
{
    return BaseT::distanceFiber(xStart, xDest);
}

double BundleSpaceMetricShortestPath::distanceBase(
    const Configuration *xStart, 
    const Configuration *xDest)
{
    OMPL_WARN("Computing geodesic distance on base space. If you want the graph-based distance, call distanceBundle instead.");
    return BaseT::distanceBase(xStart, xDest);
}


std::vector<const Configuration*> 
BundleSpaceMetricShortestPath::getInterpolationPath(
    const Configuration *xStart, 
    const Configuration *xDest)
{
  BundleSpaceGraph *parent = dynamic_cast<BundleSpaceGraph*>(bundleSpaceGraph_->getParent());

  const base::SpaceInformationPtr &base = bundleSpaceGraph_->getBase();
  const base::SpaceInformationPtr &bundle = bundleSpaceGraph_->getBundle();

  base::State *sStart = xStart->state;
  base::State *sDest = xDest->state;

  //(1) project onto base
  bundleSpaceGraph_->projectBase(sStart, xBaseStartTmp_);
  bundleSpaceGraph_->projectBase(sDest, xBaseDestTmp_);

  Configuration *xBaseStart = new Configuration(base, xBaseStartTmp_);
  Configuration *xBaseDest = new Configuration(base, xBaseDestTmp_);
  base::State *sBaseStart = xBaseStart->state;
  base::State *sBaseDest = xBaseDest->state;

  //(2) get nearest graph nodes on base
  const Configuration *xBaseNearestStart = parent->nearest( xBaseStart );
  const Configuration *xBaseNearestDest = parent->nearest( xBaseDest );

  //(3) compute path on base between nearest graph nodes
  // std::vector<const Configuration*> 
  base::PathPtr pathBasePtr = 
    parent->getPath(xBaseNearestStart->index, xBaseNearestDest->index);

  //(4) use path on base space to connect start to dest
  std::vector<const Configuration*> pathBundle;
  pathBundle.push_back(xStart);

  if(pathBasePtr){
      PathGeometricPtr gpath = std::static_pointer_cast<PathGeometric>(pathBasePtr);
      const std::vector<base::State*> pathBase = gpath->getStates();
      if(pathBase.size() > 1)
      {
        //(4b) interpolate path on base, then lift up by interpolating along fiber
         double lengthBasePath = 0;
         std::vector<double> lengthsBasePath;
         double lengthFirstSegment = base->distance(sBaseStart, pathBase.at(0));
         lengthsBasePath.push_back(lengthFirstSegment); 
         lengthBasePath += lengthFirstSegment;
         for(uint k = 1; k < pathBase.size(); k++){
             double lengthKthSegment = base->distance(pathBase.at(k-1), pathBase.at(k));
             lengthsBasePath.push_back(lengthKthSegment); 
             lengthBasePath += lengthKthSegment;
         }
         double lengthLastSegment = base->distance(pathBase.back(), sBaseDest);
         lengthsBasePath.push_back(lengthLastSegment); 
         lengthBasePath += lengthLastSegment;

         if(bundleSpaceGraph_->getFiberDimension() > 0)
         {
            //Case 1: Interpolate on Fiber
            const base::SpaceInformationPtr &fiber = bundleSpaceGraph_->getFiber();
            bundleSpaceGraph_->projectFiber(sStart, xFiberStartTmp_);
            bundleSpaceGraph_->projectFiber(sDest, xFiberDestTmp_);

            double lengthCurrent = 0;
            for(uint k = 0; k < pathBase.size(); k++){

                base::State *xFiberK = fiber->allocState();
                lengthCurrent += lengthsBasePath.at(k);

                double interpLength = lengthCurrent / lengthBasePath;
                fiber->getStateSpace()->interpolate(xFiberStartTmp_, xFiberDestTmp_, interpLength, xFiberK);

                Configuration *xk = new Configuration(bundle);
                bundleSpaceGraph_->mergeStates(pathBase.at(k), xFiberK, xk->state);
                pathBundle.push_back(xk);
                fiber->freeState(xFiberK);
            }

         }else{
             //Case 2: bundle = base, just copy states
             for(uint k = 0; k < pathBase.size(); k++){
                 Configuration *xk = new Configuration(bundle);
                 bundle->copyState(xk->state, pathBase.at(k));
                 pathBundle.push_back(xk);
             }
         }
      }
  }
  pathBundle.push_back(xDest);
  return pathBundle;
}
