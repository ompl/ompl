#include <ompl/multilevel/datastructures/ProjectionComponent.h>
#include <ompl/util/Exception.h>

using namespace ompl::multilevel;

ProjectionComponent::ProjectionComponent(ompl::base::StateSpacePtr bundleSpace, ompl::base::StateSpacePtr baseSpace)
  : bundleSpace_(bundleSpace), baseSpace_(baseSpace)
{
}

unsigned int ProjectionComponent::getBaseDimension() const
{
    if (baseSpace_)
        return baseSpace_->getDimension();
    else
        return 0;
}

unsigned int ProjectionComponent::getDimension() const
{
    return bundleSpace_->getDimension();
}

unsigned int ProjectionComponent::getCoDimension() const
{
    return getDimension() - getBaseDimension();
}

ompl::base::StateSpacePtr ProjectionComponent::getBundleSpace() const
{
    return bundleSpace_;
}

ompl::base::StateSpacePtr ProjectionComponent::getBaseSpace() const
{
    return baseSpace_;
}

ProjectionComponentType ProjectionComponent::getType() const
{
    return type_;
}

void ProjectionComponent::setType(ProjectionComponentType &type)
{
    type_ = type;
}

std::string ProjectionComponent::stateTypeToString(ompl::base::StateSpacePtr space) const
{
    std::string tstr;
    int type = space->getType();
    if (type == base::STATE_SPACE_REAL_VECTOR)
    {
        int N = space->getDimension();
        tstr = "R";
        tstr += std::to_string(N);
    }
    else if (type == base::STATE_SPACE_SE2)
    {
        tstr = "SE2";
    }
    else if (type == base::STATE_SPACE_SE3)
    {
        tstr = "SE3";
    }
    else if (type == base::STATE_SPACE_SO2)
    {
        tstr = "SO2";
    }
    else if (type == base::STATE_SPACE_SO3)
    {
        tstr = "SO3";
    }
    else if (type == base::STATE_SPACE_TIME)
    {
        tstr = "T";
    }
    else if (space->isCompound())
    {
        base::CompoundStateSpace *space_compound = space->as<base::CompoundStateSpace>();
        const std::vector<base::StateSpacePtr> space_decomposed = space_compound->getSubspaces();

        for (unsigned int k = 0; k < space_decomposed.size(); k++)
        {
            base::StateSpacePtr s0 = space_decomposed.at(k);
            tstr = tstr + stateTypeToString(s0);
            if (k < space_decomposed.size() - 1)
                tstr += "x";
        }
    }
    else
    {
        throw Exception("Unknown State Space");
    }
    return tstr;
}

std::string ProjectionComponent::getTypeAsString() const
{
    if (baseSpace_)
    {
        std::string tstr = getBundleTypeAsString() + " -> " + getBaseTypeAsString();
        if (type_ == BUNDLE_SPACE_CONSTRAINED_RELAXATION)
        {
            tstr += " (relaxation)";
        }
        else if (type_ == BUNDLE_SPACE_IDENTITY_PROJECTION)
        {
            tstr += " (identity)";
        }
        return tstr;
    }
    else
    {
        return getBundleTypeAsString();
    }
}

std::string ProjectionComponent::getBaseTypeAsString() const
{
    if (baseSpace_)
        return stateTypeToString(baseSpace_);
    else
        return "None";
}

std::string ProjectionComponent::getBundleTypeAsString() const
{
    return stateTypeToString(bundleSpace_);
}

void ProjectionComponent::print(std::ostream &out) const
{
    out << getTypeAsString() << std::endl;
}

namespace ompl
{
    namespace multilevel
    {
        std::ostream &operator<<(std::ostream &out, const ProjectionComponent &ProjectionComponent)
        {
            ProjectionComponent.print(out);
            return out;
        }
    }
}
