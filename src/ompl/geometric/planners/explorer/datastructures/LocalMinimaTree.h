#ifndef OMPL_GEOMETRIC_PLANNERS_EXPLORER_LOCALMINIMATREE_
#define OMPL_GEOMETRIC_PLANNERS_EXPLORER_LOCALMINIMATREE_

namespace ompl
{
    namespace geometric
    {
        class LocalMinimaVertex{
        };

        class LocalMinimaTree{
          public:
            LocalMinimaTree(const LocalMinimaTree &) = delete;
            LocalMinimaTree(std::vector<base::SpaceInformationPtr>);
            ~LocalMinimaTree(); 
          protected:
            std::vector<base::SpaceInformationPtr> bundle;
        };

    }
}
