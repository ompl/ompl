#include "../BundleSpaceComponent.h"

ompl::geometric::BundleSpaceComponent::BundleSpaceComponent(
                base::StateSpacePtr BundleSpace,
                base::StateSpacePtr BaseSpace):
  BundleSpace_(BundleSpace), BaseSpace_(BaseSpace)
{
}

void ompl::geometric::BundleSpaceComponent::initFiberSpace()
{
  FiberSpace_ = computeFiberSpace();
}

ompl::base::StateSpacePtr ompl::geometric::BundleSpaceComponent::getFiberSpace() const
{
  return FiberSpace_;
}

unsigned int ompl::geometric::BundleSpaceComponent::getFiberDimension() const
{
  return FiberSpace_->getDimension();
}

unsigned int ompl::geometric::BundleSpaceComponent::getBaseDimension() const
{
  return BaseSpace_->getDimension();
}

unsigned int ompl::geometric::BundleSpaceComponent::getDimension() const
{
  return BundleSpace_->getDimension();
}

ompl::geometric::BundleSpaceComponentType 
ompl::geometric::BundleSpaceComponent::getType() const
{
  return type_;
}
