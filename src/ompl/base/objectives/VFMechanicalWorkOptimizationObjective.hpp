/**
 * VFMechanicalWorkOptimizationObjective.hpp
 * COMP 450 Project 5
 * 25 November 2014 
 * Caleb Voss (cav2) & Wilson Beebe (wsb1)
 */

#ifndef V_F_MECHANICAL_WORK_OPTIMIZATION_OBJECTIVE_
#define V_F_MECHANICAL_WORK_OPTIMIZATION_OBJECTIVE_

#include "ompl/base/objectives/MechanicalWorkOptimizationObjective.h"
#include "ompl/geometric/planners/rrt/VFRRT.h"

namespace ompl
{
    namespace base
    {

        /**
         * Optimization objective that computes mechanical work between two states.
         */
        class VFMechanicalWorkOptimizationObjective : public ompl::base::MechanicalWorkOptimizationObjective
        {
    
        public:
    
            /** Constructor. */
            VFMechanicalWorkOptimizationObjective(const ompl::base::SpaceInformationPtr &si, const geometric::VFRRT::VectorField *vf)
                : ompl::base::MechanicalWorkOptimizationObjective(si), vf(*vf), sstate1(si_), sstate2(si_), d(sstate1.reals().size()), qprime(d)
            {
            }
    
            /** Assume we can always do better. */
            bool isSatisfied(ompl::base::Cost c) const
            {
                return false;
            }
    
            /** Compute mechanical work between two states. */
            ompl::base::Cost motionCost(const ompl::base::State *s1, const ompl::base::State *s2) const
            {
                // Per equation 7 in the paper
                Eigen::VectorXd f = vf(s2);
                sstate1 = s1;
                sstate2 = s2;
                for (int i = 0; i < d; i++)
                {
                    qprime[i] = sstate2[i] - sstate1[i];
                }
                // Don't included negative work
                double positiveCostAccrued = std::max((-f).dot(qprime), 0.0);
                return ompl::base::Cost(positiveCostAccrued + pathLengthWeight_*si_->distance(s1,s2));
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
    
        };
    }
}

#endif
