#include <ompl/multilevel/datastructures/Projection.h>
#include <ompl/multilevel/datastructures/ProjectionComponent.h>

using namespace ompl::base;
using namespace ompl::multilevel;

Projection::Projection(std::vector<ProjectionComponentPtr> components):
  components_(components)
{
}

void Projection::lift(const State *xBase, State *xBundle) const
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

void Projection::project(const State *xBundle, State *xBase) const
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

unsigned int Projection::getDimension() const
{
    if(components_.size() > 0)
    {
        return components_.front()->getDimension();
    }else
    {
        return 0;
    }
}

unsigned int Projection::getCoDimension() const
{
    if(components_.size() > 0)
    {
        return components_.front()->getCoDimension();
    }else
    {
        return 0;
    }
}
