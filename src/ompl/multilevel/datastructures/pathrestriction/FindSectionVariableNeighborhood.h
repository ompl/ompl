#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_PATH_RESTRICTION_FIND_SECTION_VARIABLE_NBH_
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_PATH_RESTRICTION_FIND_SECTION_VARIABLE_NBH_
#include <ompl/multilevel/datastructures/BundleSpaceGraph.h>
#include <ompl/multilevel/datastructures/pathrestriction/FindSection.h>
#include <ompl/multilevel/datastructures/ParameterSmoothStep.h>

namespace ompl
{
    namespace multilevel
    {
        using Configuration = ompl::multilevel::BundleSpaceGraph::Configuration;

        class FindSectionVariableNeighborhood: public FindSection
        {
            using BaseT = FindSection;
        public:

            FindSectionVariableNeighborhood() = delete;
            FindSectionVariableNeighborhood(PathRestriction*);

            virtual ~FindSectionVariableNeighborhood();

            virtual bool solve(BasePathHeadPtr& head) override;

            bool variableNeighborhoodPatternSearch(
                BasePathHeadPtr& head,
                bool interpolateFiberFirst = true,
                int depth = 0);

            bool sideStepAlongFiber(
                Configuration* &xOrigin, 
                base::State *state);

        protected:
            base::State* xBaseFixed_;

            ParameterSmoothStep neighborhoodBaseSpace_;
            ParameterSmoothStep neighborhoodFiberSpace_;

        };
    }
}

#endif
