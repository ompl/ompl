#include <ompl/geometric/planners/multilevel/datastructures/metrics/Reachability.h>
#include <ompl/control/Control.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SpaceInformation.h>

using namespace ompl::geometric;
using Configuration = ompl::geometric::BundleSpaceGraph::Configuration;

BundleSpaceMetricReachability::BundleSpaceMetricReachability(
    BundleSpaceGraph* bundleSpaceGraph):
  BaseT(bundleSpaceGraph)
{
    siC_ = std::dynamic_pointer_cast<ompl::control::SpaceInformation>(bundleSpaceGraph_->getBundle());

    if(!siC_)
    {
        OMPL_ERROR("SpaceInformationPtr is not dynamic.");
        throw Exception("Wrong SpaceInformationPtr");
    }

    minControlDuration_ = siC_->getMinControlDuration();
    controlSpace_ = siC_->getControlSpace();

    checkActiveControlDimensions(controlSpace_.get());
    OMPL_DEBUG("Number of Configurations in Reachable Set is %d.", numberConfigurationsReachableSet_);

    std::vector<int> empty;
    computeControl(activeControlDimensions_.size(), empty);

    ompl::control::Control *control = siC_->allocControl();
    siC_->nullControl(control);
    controls_.push_back(control);
    siC_->printControl(control);

    stepSizeReachableSet_ = 2;

    // ompl::control::Control *ctrl = siC_->allocControl();
    // createControls(controlSpace_.get(), ctrl);
    // siC_->printControl(ctrl);
    // controls_.push_back(ctrl);
}

void BundleSpaceMetricReachability::computeControl(int dimensions, std::vector<int> controlIdx)
{
  if(dimensions > 0)
  {
    std::vector<int> control1;
    control1.push_back(1);
    control1.insert(control1.end(), controlIdx.begin(), controlIdx.end());
    computeControl(dimensions-1, control1);

    std::vector<int> control0;
    control0.push_back(0);
    control0.insert(control0.end(), controlIdx.begin(), controlIdx.end());
    computeControl(dimensions-1, control0);
    return;
  }

  ompl::control::Control *control = siC_->allocControl();
  siC_->nullControl(control);
  createControls(controlSpace_.get(), control, controlIdx);
  controls_.push_back(control);
  
//TODO: DEBUG
  // std::cout << std::string(80, '-') << std::endl;
  // for(uint k = 0; k < controlIdx.size(); k++)
  // {
  //   std::cout << controlIdx.at(k);
  // }
  // std::cout << std::endl;
  siC_->printControl(control);
}

void BundleSpaceMetricReachability::createControls
(
    ompl::control::ControlSpace *controlSpace,
    ompl::control::Control *control,
    std::vector<int> idx
)
{
    for(uint k = 0; k < activeControlDimensions_.size(); k++)
    {
        std::pair<int, int> pair = activeControlDimensions_.at(k);
        int K = pair.first;
        int J = pair.second;
        setControlDimension(controlSpace, control, K, J, idx.at(k));
    }
}

void BundleSpaceMetricReachability::setAverageControl(ompl::control::ControlSpace *controlSpace, ompl::control::Control* ctrl)
{
    control::RealVectorControlSpace *realVectorSpace = controlSpace->as<control::RealVectorControlSpace>();
    const std::vector<double>& low_bound = realVectorSpace->getBounds().low;
    const std::vector<double>& high_bound = realVectorSpace->getBounds().high;
    double*& controls = ctrl->as<control::RealVectorControlSpace::ControlType>()->values;
    for(uint j = 0; j < low_bound.size(); j++)
    {
        controls[j] = low_bound[j] + 0.5*(high_bound[j]-low_bound[j]);
    }
}

void BundleSpaceMetricReachability::setControlDimension
(
    ompl::control::ControlSpace *controlSpace,
    ompl::control::Control *control,
    int Jdimension,
    bool lowerBound
)
{
    control::RealVectorControlSpace *realVectorSpace = controlSpace->as<control::RealVectorControlSpace>();
    const std::vector<double>& low_bound = realVectorSpace->getBounds().low;
    const std::vector<double>& high_bound = realVectorSpace->getBounds().high;
    double*& controls = control->as<control::RealVectorControlSpace::ControlType>()->values;
    controls[Jdimension] = (lowerBound? low_bound[Jdimension]:high_bound[Jdimension]);
}

void BundleSpaceMetricReachability::setControlDimension
(
    ompl::control::ControlSpace *controlSpace,
    ompl::control::Control *control,
    int Kcomponent,
    int Jdimension,
    bool lowerBound
)
{
    if(controlSpace->isCompound())
    {
        control::ControlSpace *controlSpaceK = 
          controlSpace->as<control::CompoundControlSpace>()->as<control::ControlSpace>(Kcomponent);

        control::Control *controlK = 
          static_cast<control::CompoundControl*>(control)->as<control::Control>(Kcomponent);
        setControlDimension(controlSpaceK, controlK, Jdimension, lowerBound);
    }else{
        setControlDimension(controlSpace, control, Jdimension, lowerBound);
    }
}


void BundleSpaceMetricReachability::checkActiveControlDimensions(
    ompl::control::ControlSpace *controlSpace
    )
{
    if(controlSpace->isCompound())
    {
        unsigned int K = controlSpace->as<control::CompoundControlSpace>()->getSubspaceCount();
        for(uint k = 0; k < K; k++)
        {
            control::ControlSpace *controlSpaceK = 
              controlSpace->as<control::CompoundControlSpace>()->as<control::ControlSpace>(k);

            if(controlSpaceK->isCompound())
            {
                OMPL_ERROR("Cannot handle double compounded control spaces.");
                throw Exception("NYI");
            }
            control::RealVectorControlSpace *realVectorSpace = 
              controlSpaceK->as<control::RealVectorControlSpace>();
            std::vector<double> diff = realVectorSpace->getBounds().getDifference();

            for(uint j = 0; j < diff.size()-1; j++)
            {
                if(diff.at(j) > 1e-5)
                {
                    activeControlDimensions_.push_back(std::pair<int, int>(k, j));
                }
            }
        }
    }else
    {
        control::RealVectorControlSpace *realVectorSpace = 
          controlSpace->as<control::RealVectorControlSpace>();
        std::vector<double> diff = realVectorSpace->getBounds().getDifference();

        for(uint j = 0; j < diff.size()-1; j++)
        {
            if(diff.at(j) > 1e-5)
            {
                activeControlDimensions_.push_back(std::pair<int, int>(0, j));
            }
        }
    }
    numberConfigurationsReachableSet_ = pow(2, activeControlDimensions_.size());
}
  
BundleSpaceMetricReachability::~BundleSpaceMetricReachability()
{
    for(uint k = 0; k < controls_.size(); k++)
    {
        siC_->freeControl(controls_.at(k));
    }
    controls_.clear();
}

void BundleSpaceMetricReachability::createReachableSet(Configuration *x)
{
    for(unsigned int k = 0; k < controls_.size(); k++)
    {
        Configuration *xi = new Configuration(bundleSpaceGraph_->getBundle());
        siC_->propagate(x->state, controls_.at(k), stepSizeReachableSet_, xi->state);
        x->reachableSet.push_back(xi); 
    }
}

double BundleSpaceMetricReachability::distanceBundle(
    const Configuration *xStart, 
    const Configuration *xDest)
{
    double d = BaseT::distanceBundle(xStart, xDest);

    if(xStart->reachableSet.size() <= 0)
    {
        return d;
    }

    double dmin = std::numeric_limits<double>::infinity();
    for(uint k = 0; k < xStart->reachableSet.size(); k++){
        double dk = BaseT::distanceBundle(xStart->reachableSet.at(k), xDest);
        if(dk < d && dk < dmin)
        {
            dmin = dk;
        }
    }

    return dmin;
}
