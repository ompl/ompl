#include "ompl/control/planners/ltl/World.h"
#include "ompl/util/Console.h"
#include <boost/functional/hash.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/unordered_map.hpp>
#include <string>

ompl::control::World::World(unsigned int np) : numProps_(np)
{
}

bool ompl::control::World::operator[](unsigned int i) const
{
    boost::unordered_map<unsigned int, bool>::const_iterator p = props_.find(i);
    if (p == props_.end())
        OMPL_ERROR("Proposition %u is not set in world", i);
    return p->second;
}

bool& ompl::control::World::operator[](unsigned int i)
{
    return props_[i];
}

unsigned int ompl::control::World::numProps() const
{
    return numProps_;
}

bool ompl::control::World::satisfies(const World& w) const
{
    boost::unordered_map<unsigned int, bool>::const_iterator p, q;
    for (p = w.props_.begin(); p != w.props_.end(); ++p)
    {
        q = props_.find(p->first);
        if (q == props_.end() || *q != *p)
            return false;
    }
    return true;
}

std::string ompl::control::World::formula(void) const
{
    if (props_.empty())
        return "true";
    boost::unordered_map<unsigned int, bool>::const_iterator p = props_.begin();
    std::string f = std::string(p->second ? "p" : "!p") + boost::lexical_cast<std::string>(p->first);
    ++p;
    for (; p != props_.end(); ++p)
        f += std::string(p->second ? " & p" : " & !p") + boost::lexical_cast<std::string>(p->first);
    return f;
}

const boost::unordered_map<unsigned int, bool>& ompl::control::World::props(void) const
{
    return props_;
}

bool ompl::control::World::operator==(const World& w) const
{
    return numProps_ == w.numProps_ && props_ == w.props_;
}

void ompl::control::World::clear(void)
{
    props_.clear();
}

namespace ompl
{
    namespace control
    {
        std::size_t hash_value(const ompl::control::World& w)
        {
            std::size_t hash = 0;
            boost::unordered_map<unsigned int, bool>::const_iterator p;
            for (p = w.props_.begin(); p != w.props_.end(); ++p)
                boost::hash_combine(hash, *p);
            return hash;
        }
    }
}
