#include <ompl/geometric/planners/quotientspace/datastructures/pathrestriction/PathRestriction.h>
#include <ompl/geometric/planners/quotientspace/datastructures/graphsampler/GraphSampler.h>

#include <ompl/base/Path.h>
#include <ompl/geometric/PathGeometric.h>

ompl::geometric::BundleSpacePathRestriction::BundleSpacePathRestriction(BundleSpaceGraph* bundleSpaceGraph):
  bundleSpaceGraph_(bundleSpaceGraph)
{
    if(bundleSpaceGraph_->isDynamic())
    {
        OMPL_ERROR("NYI: computing path sections for dynamical systems.");
        throw Exception("NYI");
    }
    if(bundleSpaceGraph_->getFiberDimension() > 0)
    {
        xFiberStart_ = bundleSpaceGraph_->getFiber()->allocState();
        xFiberGoal_ = bundleSpaceGraph_->getFiber()->allocState();
        xFiberTmp_ = bundleSpaceGraph_->getFiber()->allocState();
    }
}

ompl::geometric::BundleSpacePathRestriction::~BundleSpacePathRestriction()
{
    if(bundleSpaceGraph_->getFiberDimension() > 0)
    {
        bundleSpaceGraph_->getFiber()->freeState(xFiberStart_);
        bundleSpaceGraph_->getFiber()->freeState(xFiberGoal_);
        bundleSpaceGraph_->getFiber()->freeState(xFiberTmp_);
    }
}

void ompl::geometric::BundleSpacePathRestriction::setBasePath(ompl::base::PathPtr path)
{
    PathGeometricPtr geometricBasePath = std::static_pointer_cast<PathGeometric>(path);
    setBasePath(geometricBasePath->getStates());

}

void ompl::geometric::BundleSpacePathRestriction::setBasePath(
    std::vector<base::State*> basePath)
{
    if(bundleSpaceGraph_->isDynamic())
    {
      OMPL_WARN("NYI: computing path sections for dynamical systems.");
      return;
    }

    basePath_ = basePath;

    lengthBasePath_ = 0.0;
    intermediateLengthsBasePath_.clear();
    for(uint k = 1; k < basePath_.size(); k++){
      double lk = 
        bundleSpaceGraph_->getBase()->distance(
            basePath_.at(k-1), 
            basePath_.at(k));
      intermediateLengthsBasePath_.push_back(lk);
      lengthBasePath_ += lk;
    }
    OMPL_INFORM("Set new base path with %d states and length %f.", 
        basePath_.size(), 
        lengthBasePath_);
}

std::vector<ompl::base::State*> 
ompl::geometric::BundleSpacePathRestriction::interpolateSectionL1(
      const base::State* xFiberStart,
      const base::State* xFiberGoal) 
{
    std::vector<base::State*> basePath = basePath_;

    //Two goal states (to interpolate along goal fiber)
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

std::vector<ompl::base::State*> 
ompl::geometric::BundleSpacePathRestriction::interpolateSectionL1_FiberFirst(
      const base::State* xFiberStart,
      const base::State* xFiberGoal) 
{
    std::vector<base::State*> basePath = basePath_;

    //Two start states (to interpolate along start fiber)
    basePath.insert(basePath.begin(), basePath.front());

    std::vector<base::State*> bundlePath;
    bundlePath.resize(basePath.size());
    bundleSpaceGraph_->getBundle()->allocStates(bundlePath);

    if(bundleSpaceGraph_->getFiberDimension() > 0)
    {
        for(uint k = 0; k < basePath.size(); k++)
        {
            if(k > 0){
                bundleSpaceGraph_->liftState(basePath.at(k), xFiberGoal, bundlePath.at(k));
            }else{
                bundleSpaceGraph_->liftState(basePath.at(k), xFiberStart, bundlePath.at(k));
            }
        }

    }else{
        for(uint k = 0; k < basePath.size(); k++){
            bundleSpaceGraph_->getBundle()->copyState(bundlePath.at(k), basePath.at(k));
        }
    }
    return bundlePath;
}

std::vector<ompl::base::State*> 
ompl::geometric::BundleSpacePathRestriction::interpolateSectionL2(
      const base::State* xFiberStart,
      const base::State* xFiberGoal) 
{
    std::vector<base::State*> bundlePath;
    bundlePath.resize(basePath_.size());
    bundleSpaceGraph_->getBundle()->allocStates(bundlePath);

    if(bundleSpaceGraph_->getFiberDimension() > 0)
    {
        double lengthCurrent = 0;

        for(uint k = 0; k < basePath_.size(); k++)
        {
            double step = lengthCurrent / lengthBasePath_;
            bundleSpaceGraph_->getFiber()->getStateSpace()->interpolate(
                xFiberStart, xFiberGoal, step, xFiberTmp_);

            bundleSpaceGraph_->liftState(basePath_.at(k), xFiberTmp_, bundlePath.at(k));

            if(k < basePath_.size() - 1)
            {
                lengthCurrent += intermediateLengthsBasePath_.at(k);
            }
        }

    }else{
        for(uint k = 0; k < basePath_.size(); k++){
            bundleSpaceGraph_->getBundle()->copyState(bundlePath.at(k), basePath_.at(k));
        }
    }
    return bundlePath;
}

bool ompl::geometric::BundleSpacePathRestriction::hasFeasibleSection(
      Configuration* const xStart,
      Configuration* const xGoal) 
{
    bundleSpaceGraph_->projectFiber(xStart->state, xFiberStart_);
    bundleSpaceGraph_->projectFiber(xGoal->state, xFiberGoal_);

    //compute section
    std::vector<base::State*> bundlePathStates = 
      interpolateSectionL2(xFiberStart_, xFiberGoal_);

    //check for feasibility
    Configuration *xLast = xStart;

    std::pair<base::State*, double> lastValid;
    lastValid.first = bundleSpaceGraph_->getBundle()->allocState();

    bool found = false;
    for(uint k = 1; k < bundlePathStates.size(); k++)
    {
        if(!bundleSpaceGraph_->getBundle()->checkMotion(
              bundlePathStates.at(k-1), bundlePathStates.at(k), lastValid))
        {
            if(lastValid.second > 0)
            {
                //add last valid into the bundle graph
                Configuration *xk = new Configuration(bundleSpaceGraph_->getBundle(), lastValid.first);
                bundleSpaceGraph_->addConfiguration(xk);
                bundleSpaceGraph_->addBundleEdge(xLast, xk);
            }

            double length = std::accumulate(
                intermediateLengthsBasePath_.begin(), 
                intermediateLengthsBasePath_.begin()+(k-1), 0.0);

            length += lastValid.second * 
              bundleSpaceGraph_->getBase()->distance(
                  basePath_.at(k-1), 
                  basePath_.at(k));

            static_cast<BundleSpaceGraph*>(bundleSpaceGraph_->getParent())->getGraphSampler()->setPathBiasStartSegment(length);

            break;
        }else{
            if(k < bundlePathStates.size()-1)
            {
                Configuration *xk = new Configuration(bundleSpaceGraph_->getBundle(), bundlePathStates.at(k));
                bundleSpaceGraph_->addConfiguration(xk);
                bundleSpaceGraph_->addBundleEdge(xLast, xk);
                xLast = xk;
            }else{
                if(xGoal->index <= 0)
                {
                    bundleSpaceGraph_->vGoal_ = bundleSpaceGraph_->addConfiguration(xGoal);
                }
                bundleSpaceGraph_->addBundleEdge(xLast, xGoal);

                OMPL_DEBUG("Found feasible path section (%d edges added)", k);
                found = true;
            }
        }
    }
    bundleSpaceGraph_->getBundle()->freeState(lastValid.first);
    bundleSpaceGraph_->getBundle()->freeStates(bundlePathStates);
    return found;
}
