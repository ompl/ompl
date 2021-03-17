#include <ompl/multilevel/datastructures/Projection.h>
#include <ompl/util/Exception.h>

using namespace ompl::base;
using namespace ompl::multilevel;

Projection::Projection(ompl::base::StateSpacePtr bundleSpace, ompl::base::StateSpacePtr baseSpace)
  : bundleSpace_(bundleSpace), baseSpace_(baseSpace)
{
}

bool Projection::isAdmissible() const
{
    OMPL_WARN("NYI");
    return false;
}

unsigned int Projection::getBaseDimension() const
{
    if (baseSpace_)
    {
        return baseSpace_->getDimension();
    }else
    {
        return 0;
    }
}

unsigned int Projection::getDimension() const
{
    return bundleSpace_->getDimension();
}

unsigned int Projection::getCoDimension() const
{
    return getDimension() - getBaseDimension();
}

ompl::base::StateSpacePtr Projection::getBundle() const
{
    return bundleSpace_;
}

ompl::base::StateSpacePtr Projection::getBase() const
{
    return baseSpace_;
}

ProjectionType Projection::getType() const
{
    return type_;
}

void Projection::setType(const ProjectionType type)
{
    type_ = type;
}

std::string Projection::stateTypeToString(ompl::base::StateSpacePtr space) const
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

std::string Projection::getTypeAsString() const
{
    if (baseSpace_)
    {
        std::string tstr = getBundleTypeAsString() + " -> " + getBaseTypeAsString();
        if (type_ == PROJECTION_CONSTRAINED_RELAXATION)
        {
            tstr += " (relax)";
        }
        else if (type_ == PROJECTION_IDENTITY)
        {
            tstr += " (id)";
        }
        return tstr;
    }
    else
    {
        return getBundleTypeAsString();
    }
}

std::string Projection::getBaseTypeAsString() const
{
    if (baseSpace_)
        return stateTypeToString(baseSpace_);
    else
        return "None";
}

std::string Projection::getBundleTypeAsString() const
{
    return stateTypeToString(bundleSpace_);
}

void Projection::print(std::ostream &out) const
{
    out << getTypeAsString() << std::endl;
}

namespace ompl
{
    namespace multilevel
    {
        std::ostream &operator<<(std::ostream &out, const CompoundProjection &CompoundProjection)
        {
            CompoundProjection.print(out);
            return out;
        }
    }
}

CompoundProjection::CompoundProjection(
    base::StateSpacePtr bundleSpace, 
    base::StateSpacePtr baseSpace, 
    std::vector<ProjectionPtr>& components):
  Projection(bundleSpace, baseSpace), components_(components)
{
    setType(PROJECTION_COMPOUND);
}

void CompoundProjection::lift(const State *xBase, State *xBundle) const
{
    unsigned int M = components_.size();

    if (M > 1)
    {
        for (unsigned int m = 0; m < M; m++)
        {
            const State *xmBase = xBase->as<CompoundState>()->as<State>(m);
            State *xmBundle = xBundle->as<CompoundState>()->as<State>(m);
            components_.at(m)->lift(xmBase, xmBundle);
        }
    }
    else
    {
        components_.front()->lift(xBase, xBundle);
    }
}

void CompoundProjection::project(const State *xBundle, State *xBase) const
{
    unsigned int M = components_.size();

    if (M > 1)
    {
        for (unsigned int m = 0; m < M; m++)
        {
            if (components_.at(m)->getBaseDimension() > 0)
            {
                const State *xmBundle = xBundle->as<CompoundState>()->as<State>(m);
                State *xmBase = xBase->as<CompoundState>()->as<State>(m);
                components_.at(m)->project(xmBundle, xmBase);
            }
        }
    }
    else
    {
        components_.front()->project(xBundle, xBase);
    }
}

unsigned int CompoundProjection::getDimension() const
{
    if(components_.size() > 0)
    {
        return components_.front()->getDimension();
    }else
    {
        return 0;
    }
}

unsigned int CompoundProjection::getCoDimension() const
{
    if(components_.size() > 0)
    {
        return components_.front()->getCoDimension();
    }else
    {
        return 0;
    }
}
unsigned int CompoundProjection::getBaseDimension() const
{
    if(components_.size() > 0)
    {
        return components_.front()->getBaseDimension();
    }else
    {
        return 0;
    }
}
