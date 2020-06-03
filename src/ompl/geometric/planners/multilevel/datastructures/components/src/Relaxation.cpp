#include <ompl/geometric/planners/multilevel/datastructures/components/Relaxation.h>

ompl::geometric::BundleSpaceComponent_Relaxation::BundleSpaceComponent_Relaxation(base::StateSpacePtr BundleSpace,
                                                                                  base::StateSpacePtr BaseSpace)
  : BaseT(BundleSpace, BaseSpace)
{
}
