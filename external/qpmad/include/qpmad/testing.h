/**
    @file
    @author  Alexander Sherikov

    @copyright 2017 Alexander Sherikov. Licensed under the Apache License,
    Version 2.0. (see LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <iomanip>

namespace qpmad
{
    namespace testing
    {
        double computeObjective(const Eigen::MatrixXd &H, const Eigen::VectorXd &h, const Eigen::VectorXd &primal)
        {
            Eigen::MatrixXd L = H.triangularView<Eigen::Lower>();

            double result = 0.5 * primal.transpose() * L * L.transpose() * primal;

            if (h.rows() > 0)
            {
                result += h.transpose() * primal;
            }

            std::cout << "||| Objective = " << result << std::endl;

            return (result);
        }


        template <class t_ActiveSet, class t_ConstraintStatuses>
        void checkLagrangeMultipliers(
                const Eigen::MatrixXd &H,
                const Eigen::VectorXd &h,
                const Eigen::VectorXd &primal,
                const Eigen::MatrixXd &A,
                const t_ActiveSet &active_set,
                const MatrixIndex &num_simple_bounds,
                const t_ConstraintStatuses &constraints_status,
                const Eigen::VectorXd &dual,
                const Eigen::VectorXd &dual_direction = Eigen::VectorXd())
        {
            Eigen::MatrixXd L = H.triangularView<Eigen::Lower>();
            Eigen::VectorXd v = L * L.transpose() * primal;
            Eigen::MatrixXd M;

            if (h.rows() > 0)
            {
                v += h;
            }

            M.resize(primal.rows(), active_set.size_);

            for (MatrixIndex i = 0; i < active_set.size_; ++i)
            {
                MatrixIndex ctr_index = active_set.getIndex(i);

                if (ctr_index < num_simple_bounds)
                {
                    M.col(i).setZero();
                    switch (constraints_status[ctr_index])
                    {
                        case ConstraintStatus::ACTIVE_LOWER_BOUND:
                            M(ctr_index, i) = -1.0;
                            break;
                        case ConstraintStatus::ACTIVE_UPPER_BOUND:
                        case ConstraintStatus::EQUALITY:
                            M(ctr_index, i) = 1.0;
                            break;
                        default:
                            break;
                    }
                }
                else
                {
                    switch (constraints_status[ctr_index])
                    {
                        case ConstraintStatus::ACTIVE_LOWER_BOUND:
                            M.col(i) = -A.row(ctr_index - num_simple_bounds).transpose();
                            break;
                        case ConstraintStatus::ACTIVE_UPPER_BOUND:
                        case ConstraintStatus::EQUALITY:
                            M.col(i) = A.row(ctr_index - num_simple_bounds).transpose();
                            break;
                        default:
                            break;
                    }
                }
            }
            if (M.cols() > 0 && active_set.num_equalities_ < active_set.size_)
            {
                Eigen::HouseholderQR<Eigen::MatrixXd> dec(M);
                Eigen::VectorXd dual_check = dec.solve(-v);

                double max_diff = 0.0;
                std::cout << "===============================[Dual variables]================================="
                          << std::endl;
                for (MatrixIndex i = 0; i < active_set.size_; ++i)
                {
                    MatrixIndex ctr_index = active_set.getIndex(i);
                    std::cout << " " << i;
                    switch (constraints_status[ctr_index])
                    {
                        case ConstraintStatus::ACTIVE_LOWER_BOUND:
                            std::cout << "L ";
                            break;
                        case ConstraintStatus::ACTIVE_UPPER_BOUND:
                            std::cout << "U ";
                            break;
                        case ConstraintStatus::EQUALITY:
                            std::cout << "E ";
                            break;
                        default:
                            break;
                    }

                    std::cout << "dual " << dual(i) << " | "
                              << "ref " << dual_check(i) << " | ";


                    switch (constraints_status[ctr_index])
                    {
                        case ConstraintStatus::ACTIVE_LOWER_BOUND:
                        case ConstraintStatus::ACTIVE_UPPER_BOUND:
                            std::cout << "err " << std::abs(dual(i) - dual_check(i)) << " | ";
                            if (dual_direction.rows() > 0)
                            {
                                std::cout << "dir " << dual_direction(i) << " | "
                                          << "len " << dual(i) / dual_direction(i) << std::endl;
                            }
                            else
                            {
                                std::cout << std::endl;
                            }
                            if (max_diff < std::abs(dual(i) - dual_check(i)))
                            {
                                max_diff = std::abs(dual(i) - dual_check(i));
                            }
                            break;

                        case ConstraintStatus::EQUALITY:
                            std::cout << std::endl;
                            break;

                        default:
                            QPMAD_UTILS_THROW("This should not happen!");
                            break;
                    }
                }
                std::cout << " MAX DIFF = " << max_diff << std::endl;
                std::cout << "================================================================================"
                          << std::endl;
            }
        }


        template <class t_ActiveSet, class t_ConstraintStatuses>
        void printActiveSet(
                const t_ActiveSet &active_set,
                const t_ConstraintStatuses &constraints_status,
                const Eigen::VectorXd &dual)
        {
            std::cout << "====================================[Active set]================================"
                      << std::endl;
            for (MatrixIndex i = active_set.num_equalities_; i < active_set.size_; ++i)
            {
                MatrixIndex active_ctr_index = active_set.getIndex(i);

                std::cout << " ## " << i << " ## | Index = " << active_ctr_index
                          << " | Type = " << constraints_status[active_ctr_index] << " | Dual = " << dual(i)
                          << std::endl;
            }
            std::cout << "================================================================================"
                      << std::endl;
        }

        template <class t_Dual, class t_Indices, class t_IsLower>
        void printDualVariables(const t_Dual &dual, const t_Indices &indices, const t_IsLower &is_lower)
        {
            std::cout << "================================[Dual variables]================================"
                      << std::endl;
            QPMAD_UTILS_PERSISTENT_ASSERT(
                    dual.size() == indices.size() && dual.size() == is_lower.size(), "Inconsistent vector length");
            for (MatrixIndex i = 0; i < dual.size(); ++i)
            {
                std::cout << " ## " << i << " ## | Index = " << indices(i) << " | Lower = " << is_lower(i)
                          << " | Dual = " << dual(i) << std::endl;
            }
            std::cout << "================================================================================"
                      << std::endl;
        }
    }  // namespace testing
}  // namespace qpmad
