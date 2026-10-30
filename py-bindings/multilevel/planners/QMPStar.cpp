#include "detail.hpp"

#include "ompl/multilevel/planners/qmp/QMPStar.h"
#include "../init.h"

namespace ml = ompl::multilevel;

void ompl::binding::multilevel::initPlanners_QMPStar(nanobind::module_ &m)
{
    ompl::binding::multilevel::detail::bindBundleSpaceSequencePlanner<ml::QMPStar>(m, "QMPStar");
}
