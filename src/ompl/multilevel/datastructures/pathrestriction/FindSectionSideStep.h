#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_PATH_RESTRICTION_FIND_SECTION_SIDESTEP_
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_PATH_RESTRICTION_FIND_SECTION_SIDESTEP_
#include <ompl/multilevel/datastructures/pathrestriction/FindSection.h>

namespace ompl
{
    namespace multilevel
    {
        class FindSectionSideStep: public FindSection
        {
            using BaseT = FindSection;
        public:

            FindSectionSideStep() = delete;
            FindSectionSideStep(PathRestriction*);

            virtual ~FindSectionSideStep();

            virtual bool solve(BasePathHeadPtr& head) override;

            bool recursiveSideStep(
                BasePathHeadPtr& head,
                bool interpolateFiberFirst = true,
                unsigned int depth = 0);

        };
    }
}

#endif

