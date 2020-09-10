#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_PATH_RESTRICTION_FIND_SECTION_PATTERNDANCE_
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_PATH_RESTRICTION_FIND_SECTION_PATTERNDANCE_
#include <ompl/multilevel/datastructures/BundleSpaceGraph.h>
#include <ompl/multilevel/datastructures/pathrestriction/FindSection.h>
#include <ompl/multilevel/datastructures/ParameterSmoothStep.h>

namespace ompl
{
    namespace multilevel
    {
        using Configuration = ompl::multilevel::BundleSpaceGraph::Configuration;

        class FindSectionPatternDance: public FindSection
        {
            using BaseT = FindSection;
        public:

            FindSectionPatternDance() = delete;
            FindSectionPatternDance(PathRestriction*);

            virtual ~FindSectionPatternDance();

            virtual bool solve(BasePathHeadPtr& head) override;

            bool recursivePatternSearch(
                BasePathHeadPtr& head,
                bool interpolateFiberFirst = true,
                unsigned int depth = 0);


            bool sideStepAlongFiber(
                Configuration* &xOrigin, 
                base::State *state);

            bool wriggleFree(BasePathHeadPtr& head);

            bool tunneling(BasePathHeadPtr& head);

        protected:

            base::State* xBaseFixed_;

        };
    }
}

#endif

