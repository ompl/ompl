/**
 * VFUpstreamCriterionOptimizationObjective.hpp
 * COMP 450 Project 5
 * 25 November 2014 
 * Caleb Voss (cav2) & Wilson Beebe (wsb1)
 */


#ifndef V_F_UPSTREAM_CRITERION_OPTIMIZATION_OBJECTIVE_
#define V_F_UPSTREAM_CRITERION_OPTIMIZATION_OBJECTIVE_

#include "ompl/base/OptimizationObjective.h"
#include "ompl/geometric/planners/rrt/VFRRT.h"

namespace ompl
{
    namespace base
    {

        /**
         * Optimization objective that computes the upstream criterion between two states.
         */
        class VFUpstreamCriterionOptimizationObjective : public ompl::base::OptimizationObjective
        {
    
        public:
    
            /** Constructor. */
            VFUpstreamCriterionOptimizationObjective(const ompl::base::SpaceInformationPtr &si, const geometric::VFRRT::VectorField *vf)
                : ompl::base::OptimizationObjective(si), vf(*vf), sstate1(si_), sstate2(si_), d(sstate1.reals().size()), qprime(d)
            {
                description_ = "Upstream Criterion";
            }
    
            /** Assume we can always do better. */
            bool isSatisfied(ompl::base::Cost c) const
            {
                return false;
            }
    
            /** Compute upstream criterion between two states. */
            ompl::base::Cost motionCost(const State *s1, const State *s2) const
            {
                // Per equation 1 in the paper, Riemann approximation on the left
                sstate1 = s1;
                sstate2 = s2;
                for (int i = 0; i < d; i++)
                {
                    qprime[i] = sstate2[i] - sstate1[i];
                }
                int segments = std::ceil(si_->distance(s1,s2) / si_->getStateValidityCheckingResolution());
                si_->getMotionStates(s1, s2, interp, segments-1, true, true);
                qprime.normalize();
                double cost = 0;
                for (int i = 0; i < segments; i++)
                {
                    Eigen::VectorXd f = vf(interp[i]);
                    cost -= si_->distance(interp[i],interp[i+1])*(f.norm() - f.dot(qprime));
                    si_->freeState(interp[i]);
                }
                si_->freeState(interp[interp.size()-1]);
                interp.clear();
                std::cout << cost << "\n";
                return ompl::base::Cost(cost);
            }
    
            bool isSymmetric(void) const
            {
                return false;
            }

        private:
    
            /** VectorField associated with the space. */
            const geometric::VFRRT::VectorField &vf;
    
            /** Variables used in computation that we keep around to save on allocations. */
            mutable ompl::base::ScopedState<> sstate1;
            mutable ompl::base::ScopedState<> sstate2;
            const int d;
            mutable Eigen::VectorXd qprime;
            mutable std::vector<ompl::base::State*> interp;
    
        };

    }
}

#endif
