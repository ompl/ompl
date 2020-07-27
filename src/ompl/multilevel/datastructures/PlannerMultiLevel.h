#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_PLANNERMULTILEVEL_
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_PLANNERMULTILEVEL_
#include <ompl/base/Planner.h>

namespace ompl
{
    namespace multilevel
    {
        /** \brief MultiLevel Planner Interface
         */
        class PlannerMultiLevel : public ompl::base::Planner
        {
            using BaseT = ompl::base::Planner;

        public:
            PlannerMultiLevel(std::vector<ompl::base::SpaceInformationPtr> &siVec,
                                std::string type = "PlannerMultiLevel");
            PlannerMultiLevel(ompl::base::SpaceInformationPtr si) = delete;
            PlannerMultiLevel(ompl::base::SpaceInformationPtr si, std::string type) = delete;

            virtual ~PlannerMultiLevel();

            const ompl::base::ProblemDefinitionPtr &getProblemDefinition(int level) const;

            const std::vector<ompl::base::ProblemDefinitionPtr> &getProblemDefinitionVector() const;

            virtual void clear() override;

            /** \brief Number of BundleSpaces */
            int getLevels() const;

            /** \brief Get all dimensions of the BundleSpaces in the sequence */
            std::vector<int> getDimensionsPerLevel() const;

        protected:
            /** \brief Solution paths on each BundleSpace*/
            std::vector<ompl::base::PathPtr> solutions_;

            /** \brief Sequence of ProblemDefinitionPtr */
            std::vector<ompl::base::ProblemDefinitionPtr> pdefVec_;

            /** \brief Each BundleSpace has a unique ompl::base::SpaceInformationPtr */
            std::vector<ompl::base::SpaceInformationPtr> siVec_;
        };
    }  // namespace multilevel
}  // namespace ompl
#endif
