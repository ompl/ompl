#ifndef OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_COMPONENT_TYPES
#define OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_COMPONENT_TYPES

namespace ompl
{
    namespace geometric
    {
        enum BundleSpaceComponentType
        {
          /** \brief ompl::geometric::BundleSpaceComponent_None */
            BUNDLE_SPACE_NO_PROJECTION = 0,

          /** \brief ompl::geometric::BundleSpaceComponent_EmptySet */
            BUNDLE_SPACE_EMPTY_SET_PROJECTION = 1,

          /** \brief ompl::geometric::BundleSpaceComponent_Identity */
            BUNDLE_SPACE_IDENTITY_PROJECTION = 2,

            BUNDLE_SPACE_CONSTRAINED_RELAXATION = 3,

          /** \brief RN \rightarrow RM, m < n */
            BUNDLE_SPACE_RN_RM = 4,

          /** \brief SE2 \rightarrow R2 */
            BUNDLE_SPACE_SE2_R2 = 5,

          /** \brief SE2RN \rightarrow R2 */
            BUNDLE_SPACE_SE2RN_R2 = 6,

          /** \brief SE2RN \rightarrow SE2 */
            BUNDLE_SPACE_SE2RN_SE2 = 7,

          /** \brief SE2RN \rightarrow SE2RM, m < n */
            BUNDLE_SPACE_SE2RN_SE2RM = 8,

          /** \brief SO2RN \rightarrow SO2 */
            BUNDLE_SPACE_SO2RN_SO2 = 9,

          /** \brief SO2RN \rightarrow SO2RM, m < n */
            BUNDLE_SPACE_SO2RN_SO2RM = 10,

          /** \brief SE3 \rightarrow R3 */
            BUNDLE_SPACE_SE3_R3 = 11,

          /** \brief SE3RN \rightarrow R3 */
            BUNDLE_SPACE_SE3RN_R3 = 12,

          /** \brief SE3RN \rightarrow SE3 */
            BUNDLE_SPACE_SE3RN_SE3 = 13,

          /** \brief SE3RN \rightarrow SE3RM, m < n */
            BUNDLE_SPACE_SE3RN_SE3RM = 14,

            BUNDLE_SPACE_UNKNOWN = 99
        };
    }
}

#endif

