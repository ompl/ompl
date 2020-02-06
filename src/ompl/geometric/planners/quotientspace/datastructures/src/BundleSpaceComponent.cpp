#include "../BundleSpaceComponent.h"

BundleSpaceComponent::BundleSpaceComponent(
                base::StateSpacePtr BundleSpace,
                base::StateSpacePtr BaseSpace):
  BundleSpace_(BundleSpace), BaseSpace_(BaseSpace)
{
    FiberSpace_ = computeFiberSpace(BundleSpace_, BaseSpace_);
}

unsigned int BundleSpaceComponent::getFiberDimension() const
{
  return FiberSpace_->getDimension();
}

unsigned int BundleSpaceComponent::getBaseDimension() const
{
  return BaseSpace_->getDimension();
}

unsigned int BundleSpaceComponent::getDimension() const
{
  return BundleSpace_->getDimension();
}

BundleSpaceComponentType BundleSpaceComponent::getType() const
{
  return type_;
}

