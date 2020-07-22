#include <ompl/multilevel/datastructures/components/Relaxation.h>

ompl::multilevel::BundleSpaceComponent_Relaxation::BundleSpaceComponent_Relaxation(base::StateSpacePtr BundleSpace,
                                                                                  base::StateSpacePtr BaseSpace)
  : BaseT(BundleSpace, BaseSpace)
{
}
