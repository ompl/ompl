#include <nanobind/nanobind.h>
#include "ompl/base/SolutionNonExistenceProof.h"
#include "init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::init_SolutionNonExistenceProof(nb::module_& m)
{
    // TODO [ob::SolutionNonExistenceProof][IMPLEMENT]
    nb::class_<ob::SolutionNonExistenceProof>(m, "SolutionNonExistenceProof")
        ;
}
