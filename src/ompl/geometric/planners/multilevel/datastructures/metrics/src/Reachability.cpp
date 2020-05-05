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
    // numberConfigurationsReachableSet_ = 10;

    siC_ = 
			std::dynamic_pointer_cast<ompl::control::SpaceInformation>(bundleSpaceGraph_->getBundle());

    if(!siC_)
    {
        OMPL_ERROR("SpaceInformationPtr is not dynamic.");
        throw Exception("Wrong SpaceInformationPtr");
    }

    minControlDuration_ = siC_->getMinControlDuration();

    ompl::control::Control *ctrl = siC_->allocControl();

    controlSpace_ = siC_->getControlSpace();

    if(controlSpace_->isCompound())
    {
      unsigned int K = controlSpace_->as<control::CompoundControlSpace>()->getSubspaceCount();
      for(uint k = 0; k < K; k++)
      {
        control::RealVectorControlSpace *controlSpaceK = 
          controlSpace_->as<control::CompoundControlSpace>()->as<control::RealVectorControlSpace>(k);

        control::Control *controlk = static_cast<control::CompoundControl*>(ctrl)->as<control::Control>(k);
        setAverageControl(controlSpaceK, controlk);
        // const std::vector<double>& low_bound = controlSpaceK->getBounds().low;
        // const std::vector<double>& high_bound = controlSpaceK->getBounds().high;
        // double*& controls = controlk->as<control::RealVectorControlSpace::ControlType>()->values;
        // for(uint j = 0; j < low_bound.size(); j++)
        // {
        //   controls[j] = low_bound[j] + 0.5*(high_bound[j]-low_bound[j]);
        // }
      }
    }else
    {

    }
    siC_->printControl(ctrl);
    controls_.push_back(ctrl);

    numberConfigurationsReachableSet_ = pow(2, numberActiveControlDimensions_);
    OMPL_WARN("Number of Configurations in ReachableSet is %d.", numberConfigurationsReachableSet_);
}

void BundleSpaceMetricReachability::setAverageControl(ompl::control::RealVectorControlSpace *controlSpace, ompl::control::Control* ctrl)
{
    const std::vector<double>& low_bound = controlSpace->getBounds().low;
    const std::vector<double>& high_bound = controlSpace->getBounds().high;
    double*& controls = ctrl->as<control::RealVectorControlSpace::ControlType>()->values;
    for(uint j = 0; j < low_bound.size(); j++)
    {
      controls[j] = low_bound[j] + 0.5*(high_bound[j]-low_bound[j]);
    }
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
        siC_->propagate(x->state, controls_.at(k), 1, xi->state);
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

    for(uint k = 0; k < xStart->reachableSet.size(); k++){
        double dk = BaseT::distanceBundle(xStart->reachableSet.at(k), xDest);
        if(dk < d)
        {
            return dk;
        }
    }

    //no configuration from reachableSet is able to move closer towards xDest
    return std::numeric_limits<double>::infinity();
}
