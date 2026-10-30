#include "detail.hpp"

#include "ompl/multilevel/planners/qmp/QMP.h"
#include "../init.h"

namespace ml = ompl::multilevel;

void ompl::binding::multilevel::initPlanners_QMP(nanobind::module_ &m)
{
    ompl::binding::multilevel::detail::bindBundleSpaceSequencePlanner<ml::QMP>(m, "QMP");
}
