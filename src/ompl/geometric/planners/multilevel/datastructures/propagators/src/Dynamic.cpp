#include <ompl/geometric/planners/multilevel/datastructures/propagators/Dynamic.h>
#include <ompl/base/DynamicalMotionValidator.h>
#include <ompl/control/SimpleDirectedControlSampler.h>


ompl::geometric::BundleSpacePropagatorDynamic::BundleSpacePropagatorDynamic(
    BundleSpaceGraph *bundleSpaceGraph):
  BaseT(bundleSpaceGraph)
{
    base::SpaceInformationPtr si = bundleSpaceGraph_->getBundle();
    siC_ = dynamic_cast<ompl::control::SpaceInformation*>(si.get());
    if(!siC_)
    {
        OMPL_ERROR("Initializing dynamical propagator, but SpaceInformationPtr is not dynamic.");
        throw Exception("Wrong SpaceInformationPtr");
    }

    siC_->setMotionValidator(std::make_shared<base::DynamicalMotionValidator>(siC_));
    siC_->setup();
    siC_->setMinMaxControlDuration(1,controlDuration);
    std::cout << "MaxControlDuration: " << controlDuration << std::endl;

    // controlSampler_ = siC_->allocDirectedControlSampler();
    controlSampler_ = std::make_shared<control::SimpleDirectedControlSampler>(siC_);
    controlSampler_->setNumControlSamples(5);

    // if (!controlSampler_)
    //     controlSampler_ = siC_->allocControlSampler();

    //@TODO: need to write allocator for simpleDirectedControlSampler
    //dCSampler->setNumControlSamples(numberOfControlSamples);
    propStepSize = siC_->getPropagationStepSize();
    prop_ = siC_->getStatePropagator();
    controlRandom_ = siC_->allocControl();

}

ompl::geometric::BundleSpacePropagatorDynamic::~BundleSpacePropagatorDynamic()
{
    siC_->freeControl(controlRandom_);
}

bool ompl::geometric::BundleSpacePropagatorDynamic::steer( 
    const Configuration *from, 
    const Configuration *to, 
    Configuration *result)
{
    unsigned int cd = controlSampler_->sampleTo(controlRandom_, from->state, to->state);

    cd = siC_->propagateWhileValid(from->state, controlRandom_, cd, result->state);

    if (cd >= siC_->getMinControlDuration())
    {
        return true;
    }	
    return false;


    ////(2) different approach (SIngle Control)
    //controlSampler_->sample(controlRandom_);
    //unsigned int cd = rng_.uniformInt(siC_->getMinControlDuration(), siC_->getMaxControlDuration());
    //unsigned int propCd = siC_->propagateWhileValid(from->state, controlRandom_, cd, result->state);
    // return (propCd == cd);
}

