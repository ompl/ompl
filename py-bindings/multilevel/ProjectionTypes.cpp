#include <nanobind/nanobind.h>

#include "ompl/multilevel/datastructures/ProjectionTypes.h"
#include "init.h"

namespace nb = nanobind;

void ompl::binding::multilevel::init_ProjectionTypes(nb::module_ &m)
{
    nb::enum_<ompl::multilevel::ProjectionType>(m, "ProjectionType")
        .value("PROJECTION_NONE", ompl::multilevel::ProjectionType::PROJECTION_NONE)
        .value("PROJECTION_EMPTY_SET", ompl::multilevel::ProjectionType::PROJECTION_EMPTY_SET)
        .value("PROJECTION_IDENTITY", ompl::multilevel::ProjectionType::PROJECTION_IDENTITY)
        .value("PROJECTION_CONSTRAINED_RELAXATION", ompl::multilevel::ProjectionType::PROJECTION_CONSTRAINED_RELAXATION)
        .value("PROJECTION_RN_RM", ompl::multilevel::ProjectionType::PROJECTION_RN_RM)
        .value("PROJECTION_SE2_R2", ompl::multilevel::ProjectionType::PROJECTION_SE2_R2)
        .value("PROJECTION_SE2RN_R2", ompl::multilevel::ProjectionType::PROJECTION_SE2RN_R2)
        .value("PROJECTION_SE2RN_SE2", ompl::multilevel::ProjectionType::PROJECTION_SE2RN_SE2)
        .value("PROJECTION_SE2RN_SE2RM", ompl::multilevel::ProjectionType::PROJECTION_SE2RN_SE2RM)
        .value("PROJECTION_SO2RN_SO2", ompl::multilevel::ProjectionType::PROJECTION_SO2RN_SO2)
        .value("PROJECTION_SO2RN_SO2RM", ompl::multilevel::ProjectionType::PROJECTION_SO2RN_SO2RM)
        .value("PROJECTION_SE3_R3", ompl::multilevel::ProjectionType::PROJECTION_SE3_R3)
        .value("PROJECTION_SE3RN_R3", ompl::multilevel::ProjectionType::PROJECTION_SE3RN_R3)
        .value("PROJECTION_SE3RN_SE3", ompl::multilevel::ProjectionType::PROJECTION_SE3RN_SE3)
        .value("PROJECTION_SE3RN_SE3RM", ompl::multilevel::ProjectionType::PROJECTION_SE3RN_SE3RM)
        .value("PROJECTION_SO3RN_SO3", ompl::multilevel::ProjectionType::PROJECTION_SO3RN_SO3)
        .value("PROJECTION_SO3RN_SO3RM", ompl::multilevel::ProjectionType::PROJECTION_SO3RN_SO3RM)
        .value("PROJECTION_RNSO2_RN", ompl::multilevel::ProjectionType::PROJECTION_RNSO2_RN)
        .value("PROJECTION_SO2N_SO2M", ompl::multilevel::ProjectionType::PROJECTION_SO2N_SO2M)
        .value("PROJECTION_TASK_SPACE", ompl::multilevel::ProjectionType::PROJECTION_TASK_SPACE)
        .value("PROJECTION_COMPOUND", ompl::multilevel::ProjectionType::PROJECTION_COMPOUND)
        .value("PROJECTION_UNKNOWN", ompl::multilevel::ProjectionType::PROJECTION_UNKNOWN);
}
