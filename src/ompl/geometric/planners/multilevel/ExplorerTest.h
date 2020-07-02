#ifndef OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_EXPLORERTEST_
#define OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_EXPLORERTEST_
#include <ompl/geometric/planners/multilevel/datastructures/BundleSpaceSequence.h>
#include <ompl/geometric/planners/multilevel/algorithms/ExplorerTestImpl.h>

namespace ompl
{
    namespace geometric
    {

        using ExplorerTest = ompl::geometric::BundleSpaceSequence<ompl::geometric::ExplorerTestImpl>;

    }  // namespace geometric
}  // namespace ompl

#endif