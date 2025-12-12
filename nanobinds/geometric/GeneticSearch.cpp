#include <nanobind/nanobind.h>
#include "ompl/geometric/GeneticSearch.h"
#include "init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::init_GeneticSearch(nb::module_& m)
{
    // TODO [og::GeneticSearch][IMPLEMENT]
    nb::class_<og::GeneticSearch>(m, "GeneticSearch")
        ;
}
