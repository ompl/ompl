#ifndef OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_TYPES
#define OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_TYPES

namespace ompl
{
    namespace geometric
    {
        enum BundleSpaceType
        {
            BUNDLE_SPACE_NO_PROJECTION = 0,
            BUNDLE_SPACE_EMPTY_SET_PROJECTION = 1,
            BUNDLE_SPACE_IDENTITY_PROJECTION = 2,
            BUNDLE_SPACE_CONSTRAINED_RELAXATION = 3,
            BUNDLE_SPACE_RN_RM = 4,
            BUNDLE_SPACE_SE2RN_R2 = 5,
            BUNDLE_SPACE_SE2RN_SE2RM = 6,
            BUNDLE_SPACE_SO2RN_SO2RM = 7,
            BUNDLE_SPACE_SE3RN_R3 = 8,
            BUNDLE_SPACE_SE3RN_SE3RM = 9
        };
    }
}

#endif

