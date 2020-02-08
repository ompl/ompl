#ifndef OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_COMPONENT_TYPES
#define OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_COMPONENT_TYPES

namespace ompl
{
    namespace geometric
    {
        enum BundleSpaceComponentType
        {
            BUNDLE_SPACE_NO_PROJECTION = 0,
            BUNDLE_SPACE_EMPTY_SET_PROJECTION = 1,
            BUNDLE_SPACE_IDENTITY_PROJECTION = 2,
            BUNDLE_SPACE_CONSTRAINED_RELAXATION = 3,
            BUNDLE_SPACE_RN_RM = 4,
            BUNDLE_SPACE_SE2_R2 = 5,
            BUNDLE_SPACE_SE2RN_R2 = 6,
            BUNDLE_SPACE_SE2RN_SE2 = 7,
            BUNDLE_SPACE_SE2RN_SE2RM = 8,
            BUNDLE_SPACE_SO2RN_SO2 = 9,
            BUNDLE_SPACE_SO2RN_SO2RM = 10,
            BUNDLE_SPACE_SE3_R3 = 11,
            BUNDLE_SPACE_SE3RN_R3 = 12,
            BUNDLE_SPACE_SE3RN_SE3 = 13,
            BUNDLE_SPACE_SE3RN_SE3RM = 14,
            BUNDLE_SPACE_UNKNOWN = 99
        };
    }
}

#endif

