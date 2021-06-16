#ifndef OMPL_MULTILEVEL_PLANNERS_BundleSpace_MultiSPARSIMPL_
#define OMPL_MULTILEVEL_PLANNERS_BundleSpace_MultiSPARSIMPL_
#include <ompl/multilevel/datastructures/BundleSpaceGraphSparse.h>
#include <ompl/datastructures/PDF.h>

namespace ompl
{
    namespace base
    {
        OMPL_CLASS_FORWARD(OptimizationObjective);
    }
    namespace multilevel
    {
        /** \brief Sparse Quotient-space roadMap Planner (MultiSPARS) Algorithm*/
        class MultiSPARSImpl : public BundleSpaceGraphSparse
        {
            using BaseT = BundleSpaceGraphSparse;

        public:
            MultiSPARSImpl(const base::SpaceInformationPtr &si, BundleSpace *parent_);

            virtual ~MultiSPARSImpl() override;

            /** \brief One iteration of SPARS using restriction sampling with
             * visibility regions */
            void grow() override;

            /** \brief Check if number of consecutive failures is larger than
             * maxFailures_ and no solution exists.*/
            bool isInfeasible() override;

            /** \brief Check if number of consecutive failures is larger than
             * maxFailures_ */
            bool hasConverged() override;

            void clear() override;
            void setup() override;

            /** \brief Return estimate of free state space coverage using the
             * formula $(1 / (M + 1))$ whereby $M$ is the number of
             * consecutive failures to sample a feasible state */
            double getImportance() const override;

        protected:

            std::vector<base::State *> randomWorkStates_;

            bool isInfeasible_{false};
        };
    }  // namespace multilevel
}  // namespace ompl

#endif
