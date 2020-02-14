#include <ompl/geometric/planners/quotientspace/datastructures/BundleSpaceComponent.h>
#include <ompl/util/Exception.h>

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
  if(FiberSpace_) return FiberSpace_->getDimension();
  else return 0;
}

unsigned int ompl::geometric::BundleSpaceComponent::getBaseDimension() const
{
  if(BaseSpace_) return BaseSpace_->getDimension();
  else return 0;
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

void 
ompl::geometric::BundleSpaceComponent::setType(BundleSpaceComponentType& type)
{
  type_ = type;
}

std::string ompl::geometric::BundleSpaceComponent::stateTypeToString(
    base::StateSpacePtr space) const
{
    std::string tstr;
    int type = space->getType();
    if(type == base::STATE_SPACE_REAL_VECTOR){
        int N = space->getDimension();
        tstr = "R";
        tstr += std::to_string(N);
    }else if(type == base::STATE_SPACE_SE2){
        tstr = "SE2";
    }else if(type == base::STATE_SPACE_SE3){
        tstr = "SE3";
    }else if(type == base::STATE_SPACE_SO2){
        tstr = "SO2";
    }else if(type == base::STATE_SPACE_SO3){
        tstr = "SO3";
    }else{
      tstr = "Unknown";
      if(space->isCompound())
      {
        base::CompoundStateSpace *space_compound = space->as<base::CompoundStateSpace>();
        if(space_compound->getSubspaceCount() == 2)
        {
          const std::vector<base::StateSpacePtr> space_decomposed = space_compound->getSubspaces();
          base::StateSpacePtr s0 = space_decomposed.at(0);
          base::StateSpacePtr s1 = space_decomposed.at(1);
          tstr = stateTypeToString(s0)+"x"+stateTypeToString(s1);
        }
      }
    }
    return tstr;
}

std::string ompl::geometric::BundleSpaceComponent::getTypeAsString() const
{
    if(BaseSpace_){
      std::string tstr = getBundleTypeAsString() + " -> " + getBaseTypeAsString();
      if(type_ == BUNDLE_SPACE_CONSTRAINED_RELAXATION){
        tstr += " (relaxation)";
      }else if(type_ == BUNDLE_SPACE_IDENTITY_PROJECTION){
        tstr += " (identity)";
      }
      return tstr;
    }else{
        return getBundleTypeAsString();
    }
}

std::string ompl::geometric::BundleSpaceComponent::getFiberTypeAsString() const
{
  if(FiberSpace_) return stateTypeToString(FiberSpace_);
  else return "None";
}

std::string ompl::geometric::BundleSpaceComponent::getBaseTypeAsString() const
{
  if(BaseSpace_) return stateTypeToString(BaseSpace_);
  else return "None";
}

std::string ompl::geometric::BundleSpaceComponent::getBundleTypeAsString() const
{
    return stateTypeToString(BundleSpace_);
}

void ompl::geometric::BundleSpaceComponent::print(std::ostream &out) const
{
  out << getTypeAsString() << std::endl;
}
namespace ompl
{
    namespace geometric
    {
        std::ostream &operator<<(std::ostream &out, const BundleSpaceComponent &bundleSpaceComponent)
        {
            bundleSpaceComponent.print(out);
            return out;
        }
    } 
}
