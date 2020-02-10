#ifndef OMPL_GEOMETRIC_PLANNERS_QUOTIENTSPACE_SQMPIMPL_
#define OMPL_GEOMETRIC_PLANNERS_QUOTIENTSPACE_SQMPIMPL_
#include <ompl/geometric/planners/quotientspace/datastructures/QuotientSpaceGraphSparse.h>
#include <ompl/datastructures/PDF.h>

namespace ompl
{
    namespace base
    {
        OMPL_CLASS_FORWARD(OptimizationObjective);
    }
    namespace geometric
    {
        /** \brief Implementation of QuotientSpace Rapidly-Exploring Random Trees Algorithm*/
        class SQMPImpl : public ompl::geometric::QuotientSpaceGraphSparse
        {
            using BaseT = QuotientSpaceGraphSparse;

        public:
            SQMPImpl(const ompl::base::SpaceInformationPtr &si, QuotientSpace *parent_);
            virtual ~SQMPImpl() override;
            /** \brief One iteration of RRT with adjusted sampling function */
            virtual void grow() override;
            void expand(); // for pdf sampling as in PRM
            virtual bool getSolution(ompl::base::PathPtr &solution) override;
            /** \brief Importance based on how many vertices the tree has */
            double getImportance() const override;
            /** \brief Uniform sampling */
            virtual bool sample(ompl::base::State *q_random) override;
            /** \brief \brief Quotient-Space sampling by choosing a random vertex from parent
                class tree */
            virtual bool sampleQuotient(ompl::base::State *) override;

            virtual void setup() override;
            virtual void clear() override;

            void setGoalBias(double goalBias);
            double getGoalBias() const;
            void setRange(double distance);
            double getRange() const;

            void addMileStone(Configuration *q_random);
            Configuration * addConfigurationDense(Configuration *q_random);
            bool getPlannerTerminationCondition();
        protected:
            /** \brief Random configuration placeholder */
            Configuration *qRandom_{nullptr};
            /** \brief Current shortest path on tree */
            std::vector<Vertex> shortestPathVertices_;

            /** \brief Maximum distance of expanding the tree */
            double maxDistance_{.0};
            /** \brief Goal bias similar to RRT */
            double goalBias_{.05};
            
            /** \brief The maximum number of failures before terminating the algorithm */
            unsigned int maxFailures_{1000u};
        };
    }  // namespace geometric
}  // namespace ompl

#endif
