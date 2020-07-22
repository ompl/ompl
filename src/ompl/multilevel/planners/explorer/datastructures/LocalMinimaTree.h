#ifndef OMPL_MULTILEVEL_PLANNERS_EXPLORER_LOCALMINIMATREE_
#define OMPL_MULTILEVEL_PLANNERS_EXPLORER_LOCALMINIMATREE_

#include <vector>

namespace ompl
{
    namespace base
    {
        OMPL_CLASS_FORWARD(SpaceInformation);
    }
    namespace multilevel
    {
      
        class LocalMinimaNode
        {
          public:
            double getLength();
          private:
            //content
            double length_;
            std::vector<LocalMinimaVertex*> children_;
        };

        //The PlannerData class for Local minima tree
        class LocalMinimaTree
        {
          public:
            LocalMinimaTree(const LocalMinimaTree &) = delete;
            LocalMinimaTree(std::vector<base::SpaceInformationPtr>);
            ~LocalMinimaTree(); 

            unsigned int getNumberOfMinima(unsigned int level);
            unsigned int getNumberOfMinima();
            unsigned int getNumberOfLevel();

            void setSelectedMinimum(std::vector<int> treePath);
            std::vector<int> getSelectedMinimum();

            /* Browser-like functionalities to select minimum (can be
             * mapped to hjkl, arrow keys or mouse buttons) */
            void setSelectedMinimumPrev();
            void setSelectedMinimumNext();
            void setSelectedMinimumCollapse();
            void setSelectedMinimumExpand();

          protected:
            //needed to convert between representations of tree
            std::vector<base::SpaceInformationPtr> siVec_; 
            std::vector<int> selectedMinimum_;
            LocalMinimaNode *root;
        };

    }
}
