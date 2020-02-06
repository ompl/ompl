#include "../BundleSpace.h"

BundleSubspace::BundleSubspace(
                base::StateSpacePtr BundleSpace,
                base::StateSpacePtr BaseSpace):
  BundleSpace_(BundleSpace), BaseSpace_(BaseSpace)
{
    FiberSpace_ = getFiberSpace(BundleSpace_, BaseSpace_);
}

unsigned int BundleSubspace::getFiberDimension() const
{
  return FiberSpace_->getDimension();
}

unsigned int BundleSubspace::getBaseDimension() const
{
  return BaseSpace_->getDimension();
}

unsigned int BundleSubspace::getDimension() const
{
  return BundleSpace_->getDimension();
}

BundleSpaceType BundleSubspace::getType() const
{
  return type_;
}

