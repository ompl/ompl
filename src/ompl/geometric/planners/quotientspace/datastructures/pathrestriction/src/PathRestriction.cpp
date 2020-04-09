#include <ompl/geometric/planners/quotientspace/datastructures/pathrestriction/PathRestriction.h>
#include <ompl/base/Path.h>
#include <ompl/geometric/PathGeometric.h>

ompl::geometric::BundleSpacePathRestriction::BundleSpacePathRestriction(BundleSpaceGraph* bundleSpaceGraph):
  bundleSpaceGraph_(bundleSpaceGraph)
{
    if(bundleSpaceGraph_->isDynamic())
    {
      OMPL_WARN("NYI: computing path sections for dynamical systems.");
      return;
    }
}
ompl::geometric::BundleSpacePathRestriction::~BundleSpacePathRestriction()
{
}

void ompl::geometric::BundleSpacePathRestriction::setBasePath(ompl::base::PathPtr path)
{
    if(bundleSpaceGraph_->isDynamic())
    {
      OMPL_WARN("NYI: computing path sections for dynamical systems.");
      return;
    }

    basePath_ = path;
}

ompl::base::PathPtr ompl::geometric::BundleSpacePathRestriction::getBasePath()
{
    return basePath_;
}

std::vector<ompl::base::State*> 
ompl::geometric::BundleSpacePathRestriction::interpolateManhattan(
      const base::State* xFiberStart,
      const base::State* xFiberGoal) 
{
    PathGeometricPtr geometricBasePath = std::static_pointer_cast<PathGeometric>(basePath_);
    std::vector<base::State*> basePath = geometricBasePath->getStates();
    basePath.push_back(basePath.back());

    std::vector<base::State*> bundlePath;
    bundlePath.resize(basePath.size());
    bundleSpaceGraph_->getBundle()->allocStates(bundlePath);

    if(bundleSpaceGraph_->getFiberDimension() > 0)
    {
        for(uint k = 0; k < basePath.size(); k++)
        {
            if(k < basePath.size()-1){
                bundleSpaceGraph_->liftState(basePath.at(k), xFiberStart, bundlePath.at(k));
            }else{
                bundleSpaceGraph_->liftState(basePath.at(k), xFiberGoal, bundlePath.at(k));
            }
        }

    }else{
        for(uint k = 0; k < basePath.size(); k++){
            bundleSpaceGraph_->getBundle()->copyState(bundlePath.at(k), basePath.at(k));
        }
    }
    return bundlePath;
}
