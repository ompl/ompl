#include "detail.hpp"

#include "ompl/multilevel/planners/qrrt/QRRTStar.h"
#include "../init.h"

namespace ml = ompl::multilevel;

void ompl::binding::multilevel::initPlanners_QRRTStar(nanobind::module_ &m)
{
    ompl::binding::multilevel::detail::bindBundleSpaceSequencePlanner<ml::QRRTStar>(m, "QRRTStar");
}
