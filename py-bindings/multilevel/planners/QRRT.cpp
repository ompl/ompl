#include "detail.hpp"

#include "ompl/multilevel/planners/qrrt/QRRT.h"
#include "../init.h"

namespace ml = ompl::multilevel;

void ompl::binding::multilevel::initPlanners_QRRT(nanobind::module_ &m)
{
    ompl::binding::multilevel::detail::bindBundleSpaceSequencePlanner<ml::QRRT>(m, "QRRT");
}
