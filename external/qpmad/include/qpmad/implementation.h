/**
    @file
    @author  Alexander Sherikov

    @copyright 2017 Alexander Sherikov. Licensed under the Apache License,
    Version 2.0. (see LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief Solver implementation
*/


#pragma once

#include "common.h"
#include "givens.h"
#include "inverse.h"
#include "solver_parameters.h"
#include "constraint_status.h"
#include "chosen_constraint.h"
#include "active_set.h"
#include "factorization_data.h"

#ifdef QPMAD_ENABLE_TRACING
#    include "testing.h"
#endif



namespace qpmad
{
    template <typename t_Scalar, int t_primal_size, int t_has_bounds, int t_general_ctr_number>
    class SolverBase
    {
    public:
        enum ReturnStatus
        {
            OK = 0,
            MAXIMAL_NUMBER_OF_ITERATIONS = 4,
            UNDEFINED = -1  // used for initialization of status variables
        };

        template <int t_rows>
        using Vector = Eigen::Matrix<t_Scalar, t_rows, 1>;
        template <int t_rows, int t_cols>
        using Matrix = Eigen::Matrix<t_Scalar, t_rows, t_cols>;  /// @deprecated
        using Scalar = t_Scalar;


    protected:
        using MatrixRef = Eigen::Ref<Eigen::Matrix<t_Scalar, Eigen::Dynamic, Eigen::Dynamic>>;
        using VectorRef = Eigen::Ref<Eigen::Matrix<t_Scalar, Eigen::Dynamic, 1>>;


    protected:
        Vector<t_primal_size> dual_;

        Vector<t_primal_size> primal_step_direction_;
        Vector<t_primal_size> dual_step_direction_;

        ActiveSet<t_primal_size> active_set_;
        FactorizationData<t_Scalar, t_primal_size> factorization_data_;

        MatrixIndex primal_size_;
        MatrixIndex h_size_;
        MatrixIndex num_simple_bounds_;
        MatrixIndex num_general_constraints_;
        MatrixIndex num_constraints_;

        const static MatrixIndex num_constraints_compile_time_ =
                Eigen::Dynamic == t_general_ctr_number ?
                        Eigen::Dynamic :
                        (0 == t_has_bounds ? t_general_ctr_number :
                                             (Eigen::Dynamic == t_primal_size ? Eigen::Dynamic :
                                                                                t_general_ctr_number + t_primal_size));


        Vector<t_general_ctr_number> general_ctr_dot_primal_;

        std::ptrdiff_t iter_counter_;
        ChosenConstraint chosen_ctr_;

        SolverParameters::HessianType hessian_type_;
        bool machinery_initialized_;

        Eigen::Array<uint8_t, num_constraints_compile_time_, 1> constraints_status_;



    public:
        SolverBase()
        {
            primal_size_ = 0;
            h_size_ = 0;
            num_simple_bounds_ = 0;
            num_general_constraints_ = 0;

            iter_counter_ = 0;
            machinery_initialized_ = false;
            hessian_type_ = SolverParameters::UNDEFINED;
        }


        /**
         * @brief Returns type of the Hessian produced by the latest execution
         * of `solve()`.
         */
        SolverParameters::HessianType getHessianType() const
        {
            return (hessian_type_);
        }


        /**
         * @brief Returns number of inequality iterations during the latest
         * execution of `solve()`.
         */
        std::ptrdiff_t getNumberOfInequalityIterations() const
        {
            return (iter_counter_);
        }


        /**
         * @brief Returns dual variables (Lagrange multipliers) corresponding
         * to inequality constraints. Must be called after successful
         * `solve()`, the result is undefined if previous call to `solve()`
         * failed.
         *
         * @tparam t_status_size
         * @tparam t_dual_size
         * @tparam t_index_size
         *
         * @param[out] dual dual variables
         * @param[out] indices constraint indices corresponding to the dual
         * variables, index 0 corresponds to the first simple bound if present
         * or to the first general constraint otherwise
         * @param[out] is_lower flags indicating if lower or upper bound is
         * active
         */
        template <int t_status_size, int t_dual_size, int t_index_size>
        void getInequalityDual(
                Vector<t_dual_size> &dual,
                Eigen::Matrix<MatrixIndex, t_index_size, 1> &indices,
                Eigen::Matrix<bool, t_status_size, 1> &is_lower) const
        {
            if (machinery_initialized_)
            {
                const MatrixIndex size = active_set_.size_ - active_set_.num_equalities_;

                dual.resize(size);
                indices.resize(size);
                is_lower.resize(size);

                for (MatrixIndex i = active_set_.num_equalities_; i < active_set_.size_; ++i)
                {
                    const std::size_t output_index = i - active_set_.num_equalities_;

                    dual(output_index) = dual_(i);
                    indices(output_index) = active_set_.getIndex(i);
                    is_lower(output_index) =
                            ConstraintStatus::ACTIVE_LOWER_BOUND == constraints_status_(indices(output_index));
                }
            }
            else
            {
                dual.resize(0);
                indices.resize(0);
                is_lower.resize(0);
            }
        }


        void reserve(
                const MatrixIndex primal_size,
                const MatrixIndex num_simple_bounds,
                const MatrixIndex num_general_constraints)
        {
            reserveMachinery(primal_size, num_general_constraints);
            constraints_status_.resize(num_simple_bounds + num_general_constraints);
            reserveDual(primal_size);
        }


    protected:
        template <class t_primal, class t_H, class t_h, class t_lb, class t_ub, class t_A, class t_Alb, class t_Aub>
        ReturnStatus solveGeneric(
                t_primal &primal,
                t_H &H,
                const t_h &h,
                const t_lb &lb,
                const t_ub &ub,
                const t_A &A,
                const t_Alb &Alb,
                const t_Aub &Aub,
                const SolverParameters &param)
        {
            QPMAD_TRACE(std::setprecision(std::numeric_limits<double>::digits10));

            machinery_initialized_ = false;
            iter_counter_ = 0;


            num_constraints_ = num_simple_bounds_ + num_general_constraints_;

            if (0 == h_size_)
            {
                // trivial unconstrained optimum
                setZero(primal);

                if (0 == num_constraints_)
                {
                    // trivial solution
                    return (OK);
                }
            }


            switch (param.hessian_type_)
            {
                case SolverParameters::HESSIAN_LOWER_TRIANGULAR:
                    factorizeCholeskyInPlace(H);
                    /* Falls through. */
                case SolverParameters::HESSIAN_CHOLESKY_FACTOR:
                    hessian_type_ = SolverParameters::HESSIAN_CHOLESKY_FACTOR;
                    // unconstrained optimum
                    if (h_size_ > 0)
                    {
                        primal = -h;
                        H.template triangularView<Eigen::Lower>().solveInPlace(primal);
                        H.transpose().template triangularView<Eigen::Upper>().solveInPlace(primal);
                    }
                    break;

                case SolverParameters::HESSIAN_INVERTED_CHOLESKY_FACTOR:
                    hessian_type_ = SolverParameters::HESSIAN_INVERTED_CHOLESKY_FACTOR;
                    // unconstrained optimum
                    if (h_size_ > 0)
                    {
                        primal_step_direction_.noalias() = H.template triangularView<Eigen::Upper>().transpose() * -h;
                        primal.noalias() = H.template triangularView<Eigen::Upper>() * primal_step_direction_;
                    }
                    break;

                default:
                    QPMAD_UTILS_THROW("Malformed solver parameters!");
                    break;
            }


            if (0 == num_constraints_)
            {
                // return unconstrained optimum
                return (OK);
            }


            // check consistency of general constraints and activate
            // equality constraints
            constraints_status_.resize(num_constraints_);
            MatrixIndex num_equalities = 0;
            for (MatrixIndex i = 0; i < num_constraints_; ++i)
            {
                chosen_ctr_.is_simple_ = i < num_simple_bounds_;

                double lb_i;
                double ub_i;

                if (true == chosen_ctr_.is_simple_)
                {
                    lb_i = lb(i);
                    ub_i = ub(i);
                }
                else
                {
                    chosen_ctr_.general_constraint_index_ = i - num_simple_bounds_;
                    lb_i = Alb(chosen_ctr_.general_constraint_index_);
                    ub_i = Aub(chosen_ctr_.general_constraint_index_);
                }


                if (lb_i - param.tolerance_ > ub_i)
                {
                    constraints_status_[i] = ConstraintStatus::INCONSISTENT;
                    QPMAD_UTILS_THROW("Inconsistent constraints (lb > ub).");
                }

                if (std::abs(lb_i - ub_i) > param.tolerance_)
                {
                    constraints_status_[i] = ConstraintStatus::INACTIVE;
                }
                else
                {
                    constraints_status_[i] = ConstraintStatus::EQUALITY;
                    ++num_equalities;


                    if (true == chosen_ctr_.is_simple_)
                    {
                        chosen_ctr_.violation_ = lb_i - primal(i);
                    }
                    else
                    {
                        chosen_ctr_.violation_ = lb_i - A.row(chosen_ctr_.general_constraint_index_) * primal;
                    }

                    initializeMachineryLazy(H, param.return_inverted_cholesky_factor_);


                    // if 'primal_size_' constraints are already activated
                    // all other constraints are linearly dependent
                    if (active_set_.hasEmptySpace())
                    {
                        double ctr_i_dot_primal_step_direction;

                        if (true == chosen_ctr_.is_simple_)
                        {
                            factorization_data_.computeEqualityPrimalStep(primal_step_direction_, i, active_set_.size_);

                            ctr_i_dot_primal_step_direction = primal_step_direction_(i);
                        }
                        else
                        {
                            factorization_data_.computeEqualityPrimalStep(
                                    primal_step_direction_,
                                    A.row(chosen_ctr_.general_constraint_index_),
                                    active_set_.size_);

                            ctr_i_dot_primal_step_direction =
                                    A.row(chosen_ctr_.general_constraint_index_) * primal_step_direction_;
                        }

                        // if step direction is a zero vector, constraint is
                        // linearly dependent with previously added constraints
                        if (ctr_i_dot_primal_step_direction < -param.tolerance_)
                        {
                            double primal_step_length_ = chosen_ctr_.violation_ / ctr_i_dot_primal_step_direction;

                            primal.noalias() += primal_step_length_ * primal_step_direction_;

                            if (false
                                == factorization_data_.update(
                                        active_set_.size_, chosen_ctr_.is_simple_, param.tolerance_))
                            {
                                QPMAD_UTILS_THROW("Failed to add an equality constraint -- is this possible?");
                            }
                            active_set_.addEquality(i);

                            continue;
                        }
                    }  // otherwise -- linear dependence

                    // this point is reached if constraint is linearly dependent

                    // check if this constraint is actually satisfied
                    if (std::abs(chosen_ctr_.violation_) > param.tolerance_)
                    {
                        // nope it is not
                        constraints_status_[i] = ConstraintStatus::INCONSISTENT;
                        QPMAD_UTILS_THROW("Infeasible equality constraints");
                    }
                    // otherwise keep going
                }
            }


            if (num_equalities == num_constraints_)
            {
                // exit early -- avoid unnecessary memory allocations
                return (OK);
            }


            ReturnStatus return_status;
            chooseConstraint(primal, lb, ub, A, Alb, Aub, param.tolerance_);


            if (std::abs(chosen_ctr_.violation_) < param.tolerance_)
            {
                // all constraints are satisfied
                return_status = OK;
            }
            else
            {
                return_status = MAXIMAL_NUMBER_OF_ITERATIONS;


                initializeMachineryLazy(H, param.return_inverted_cholesky_factor_);
                reserveDual(primal_size_);


                double chosen_ctr_dot_primal_step_direction = 0.0;

                //
                factorization_data_.computeInequalityDualStep(dual_step_direction_, chosen_ctr_, A, active_set_);
                if (active_set_.hasEmptySpace())
                {
                    // compute step direction in primal space
                    factorization_data_.computeInequalityPrimalStep(primal_step_direction_, active_set_);
                    chosen_ctr_dot_primal_step_direction =
                            getConstraintDotPrimalStepDirection(primal_step_direction_, A);
                }


                // last iteration is not counted, so iter_counter_ starts with 1.
                for (iter_counter_ = 1; (param.max_iter_ < 0) || (iter_counter_ <= param.max_iter_); ++iter_counter_)
                {
                    QPMAD_TRACE(">>>>>>>>>" << iter_counter_ << "<<<<<<<<<");
#ifdef QPMAD_ENABLE_TRACING
                    testing::computeObjective(H, h, primal);
#endif
                    QPMAD_TRACE("||| Chosen ctr index = " << chosen_ctr_.index_);
                    QPMAD_TRACE("||| Chosen ctr dual = " << chosen_ctr_.dual_);
                    QPMAD_TRACE("||| Chosen ctr violation = " << chosen_ctr_.violation_);


                    // check dual feasibility
                    MatrixIndex dual_blocking_index = primal_size_;
                    double dual_step_length = std::numeric_limits<double>::infinity();
                    for (MatrixIndex i = active_set_.num_equalities_; i < active_set_.size_; ++i)
                    {
                        if (dual_step_direction_(i) < -param.tolerance_)
                        {
                            const double dual_step_length_i = -dual_(i) / dual_step_direction_(i);
                            if (dual_step_length_i < dual_step_length)
                            {
                                dual_step_length = dual_step_length_i;
                                dual_blocking_index = i;
                            }
                        }
                    }


#ifdef QPMAD_ENABLE_TRACING
                    testing::checkLagrangeMultipliers(
                            H,
                            h,
                            primal,
                            A,
                            active_set_,
                            num_simple_bounds_,
                            constraints_status_,
                            dual_,
                            dual_step_direction_);
#endif


                    if (active_set_.hasEmptySpace()
                        // if step direction is a zero vector, constraint is
                        // linearly dependent with previously added constraints
                        && (std::abs(chosen_ctr_dot_primal_step_direction) > param.tolerance_))
                    {
                        double step_length = -chosen_ctr_.violation_ / chosen_ctr_dot_primal_step_direction;

                        QPMAD_TRACE("======================");
                        QPMAD_TRACE("||| Primal step length = " << step_length);
                        QPMAD_TRACE("||| Dual step length = " << dual_step_length);
                        QPMAD_TRACE("======================");


                        bool partial_step = false;
                        QPMAD_UTILS_ASSERT(
                                (step_length >= 0.0) && (dual_step_length >= 0.0),
                                "Non-negative step lengths expected.");
                        if (dual_step_length <= step_length)
                        {
                            step_length = dual_step_length;
                            partial_step = true;
                        }


                        primal.noalias() += step_length * primal_step_direction_;

                        dual_.segment(active_set_.num_equalities_, active_set_.num_inequalities_).noalias() +=
                                step_length
                                * dual_step_direction_.segment(
                                        active_set_.num_equalities_, active_set_.num_inequalities_);
                        chosen_ctr_.dual_ += step_length;
                        chosen_ctr_.violation_ += step_length * chosen_ctr_dot_primal_step_direction;

                        QPMAD_TRACE("||| Chosen ctr dual = " << chosen_ctr_.dual_);
                        QPMAD_TRACE("||| Chosen ctr violation = " << chosen_ctr_.violation_);


                        if ((partial_step)
                            // if violation is almost zero -- assume that a full step is made
                            && (std::abs(chosen_ctr_.violation_) > param.tolerance_))
                        {
                            QPMAD_TRACE("||| PARTIAL STEP");
                            // deactivate blocking constraint
                            constraints_status_[active_set_.getIndex(dual_blocking_index)] = ConstraintStatus::INACTIVE;

                            dropElementWithoutResize(dual_, dual_blocking_index, active_set_.size_);

                            factorization_data_.downdate(dual_blocking_index, active_set_.size_);

                            active_set_.removeInequality(dual_blocking_index);

                            // compute step direction in primal & dual space
                            factorization_data_.updateStepsAfterPartialStep(
                                    primal_step_direction_, dual_step_direction_, active_set_);
                            chosen_ctr_dot_primal_step_direction =
                                    getConstraintDotPrimalStepDirection(primal_step_direction_, A);
                        }
                        else
                        {
                            QPMAD_TRACE("||| FULL STEP");
                            // activate constraint
                            if (false
                                == factorization_data_.update(
                                        active_set_.size_, chosen_ctr_.is_simple_, param.tolerance_))
                            {
                                QPMAD_UTILS_THROW("Failed to add an inequality constraint -- is this possible?");
                            }

                            if (chosen_ctr_.is_lower_)
                            {
                                constraints_status_[chosen_ctr_.index_] = ConstraintStatus::ACTIVE_LOWER_BOUND;
                            }
                            else
                            {
                                constraints_status_[chosen_ctr_.index_] = ConstraintStatus::ACTIVE_UPPER_BOUND;
                            }
                            dual_(active_set_.size_) = chosen_ctr_.dual_;
                            active_set_.addInequality(chosen_ctr_.index_);

                            chooseConstraint(primal, lb, ub, A, Alb, Aub, param.tolerance_);

                            if (std::abs(chosen_ctr_.violation_) < param.tolerance_)
                            {
                                // all constraints are satisfied
                                return_status = OK;
                                break;
                            }

                            chosen_ctr_dot_primal_step_direction = 0.0;
                            factorization_data_.computeInequalityDualStep(
                                    dual_step_direction_, chosen_ctr_, A, active_set_);
                            if (active_set_.hasEmptySpace())
                            {
                                // compute step direction in primal & dual space
                                factorization_data_.computeInequalityPrimalStep(primal_step_direction_, active_set_);
                                chosen_ctr_dot_primal_step_direction =
                                        getConstraintDotPrimalStepDirection(primal_step_direction_, A);
                            }
                        }
                    }
                    else
                    {
                        if (dual_blocking_index == primal_size_)
                        {
                            QPMAD_UTILS_THROW("Infeasible inequality constraints.");
                        }
                        else
                        {
                            QPMAD_TRACE("======================");
                            QPMAD_TRACE("||| Dual step length = " << dual_step_length);
                            QPMAD_TRACE("======================");

                            // otherwise -- deactivate
                            dual_.segment(active_set_.num_equalities_, active_set_.num_inequalities_).noalias() +=
                                    dual_step_length
                                    * dual_step_direction_.segment(
                                            active_set_.num_equalities_, active_set_.num_inequalities_);
                            chosen_ctr_.dual_ += dual_step_length;

                            constraints_status_[active_set_.getIndex(dual_blocking_index)] = ConstraintStatus::INACTIVE;

                            dropElementWithoutResize(dual_, dual_blocking_index, active_set_.size_);

                            factorization_data_.downdate(dual_blocking_index, active_set_.size_);

                            active_set_.removeInequality(dual_blocking_index);

                            // compute step direction in primal & dual space
                            factorization_data_.updateStepsAfterPureDualStep(
                                    primal_step_direction_, dual_step_direction_, active_set_);
                            chosen_ctr_dot_primal_step_direction =
                                    getConstraintDotPrimalStepDirection(primal_step_direction_, A);
                        }
                    }
                }
            }

#ifdef QPMAD_ENABLE_TRACING
            if (machinery_initialized_)
            {
                testing::printActiveSet(active_set_, constraints_status_, dual_);

                testing::checkLagrangeMultipliers(
                        H, h, primal, A, active_set_, num_simple_bounds_, constraints_status_, dual_);
            }
            else
            {
                QPMAD_TRACE("||| NO ACTIVE CONSTRAINTS");
            }
#endif
            return (return_status);
        }


    private:
        void reserveMachinery(const MatrixIndex primal_size, const MatrixIndex num_general_constraints)
        {
            active_set_.initialize(primal_size);
            primal_step_direction_.resize(primal_size);
            general_ctr_dot_primal_.resize(num_general_constraints);

            factorization_data_.reserve(primal_size);
        }


        void reserveDual(const MatrixIndex primal_size)
        {
            dual_.resize(primal_size);
            dual_step_direction_.resize(primal_size);
        }


        template <class t_MatrixType>
        void initializeMachineryLazy(t_MatrixType &H, const bool return_inverted_cholesky_factor)
        {
            if (!machinery_initialized_)
            {
                reserveMachinery(primal_size_, num_general_constraints_);

                factorization_data_.initialize(H, hessian_type_, primal_size_, return_inverted_cholesky_factor);
                if (return_inverted_cholesky_factor)
                {
                    hessian_type_ = SolverParameters::HESSIAN_INVERTED_CHOLESKY_FACTOR;
                }

                machinery_initialized_ = true;
            }
        }


        template <
                class t_Primal,
                class t_LowerBounds,
                class t_UpperBounds,
                class t_Constraints,
                class t_ConstraintsLowerBounds,
                class t_ConstraintsUpperBounds>
        void chooseConstraint(
                const t_Primal &primal,
                const t_LowerBounds &lb,
                const t_UpperBounds &ub,
                const t_Constraints &A,
                const t_ConstraintsLowerBounds &Alb,
                const t_ConstraintsUpperBounds &Aub,
                const double tolerance)
        {
            chosen_ctr_.reset();


            for (MatrixIndex i = 0; i < num_simple_bounds_; ++i)
            {
                if (ConstraintStatus::INACTIVE == constraints_status_[i])
                {
                    checkConstraintViolation(i, lb(i), ub(i), primal(i));
                }
            }

            if ((std::abs(chosen_ctr_.violation_) < tolerance) && (num_general_constraints_ > 0))
            {
                general_ctr_dot_primal_.noalias() = A * primal;
                for (MatrixIndex i = num_simple_bounds_; i < num_constraints_; ++i)
                {
                    if (ConstraintStatus::INACTIVE == constraints_status_[i])
                    {
                        checkConstraintViolation(
                                i,
                                Alb(i - num_simple_bounds_),
                                Aub(i - num_simple_bounds_),
                                general_ctr_dot_primal_(i - num_simple_bounds_));
                    }
                }
                if (chosen_ctr_.index_ > num_simple_bounds_)
                {
                    chosen_ctr_.general_constraint_index_ = chosen_ctr_.index_ - num_simple_bounds_;
                }
            }

            chosen_ctr_.is_lower_ = (chosen_ctr_.violation_ < 0.0);
            chosen_ctr_.is_simple_ = (chosen_ctr_.index_ < num_simple_bounds_);
        }


        void checkConstraintViolation(
                const MatrixIndex i,
                const double lb_i,
                const double ub_i,
                const double ctr_i_dot_primal)
        {
            double ctr_violation_i = ctr_i_dot_primal - lb_i;
            if (ctr_violation_i < -std::abs(chosen_ctr_.violation_))
            {
                chosen_ctr_.violation_ = ctr_violation_i;
                chosen_ctr_.index_ = i;
            }
            else
            {
                ctr_violation_i = ctr_i_dot_primal - ub_i;
                if (ctr_violation_i > std::abs(chosen_ctr_.violation_))
                {
                    chosen_ctr_.violation_ = ctr_violation_i;
                    chosen_ctr_.index_ = i;
                }
            }
        }


        template <class t_VectorType, class t_MatrixType>
        double getConstraintDotPrimalStepDirection(const t_VectorType &primal_step_direction, const t_MatrixType &A)
                const
        {
            if (chosen_ctr_.is_simple_)
            {
                return (primal_step_direction(chosen_ctr_.index_));
            }
            else
            {
                return (A.row(chosen_ctr_.general_constraint_index_) * primal_step_direction);
            }
        }


        template <int... t_Other>
        static void factorizeCholeskyInPlace(Eigen::Matrix<t_Scalar, t_Other...> &H)
        {
            const Eigen::LLT<Eigen::Ref<typename std::decay<decltype(H)>::type>, Eigen::Lower> llt(H);
            QPMAD_UTILS_PERSISTENT_ASSERT(
                    Eigen::Success == llt.info(), "Could not perform Cholesky decomposition of the Hessian (dense).");
        }

        static void factorizeCholeskyInPlace(MatrixRef &H)
        {
            const Eigen::LLT<Eigen::Ref<typename std::decay<decltype(H)>::type>, Eigen::Lower> llt(H);
            QPMAD_UTILS_PERSISTENT_ASSERT(
                    Eigen::Success == llt.info(), "Could not perform Cholesky decomposition of the Hessian (dense).");
        }

#if !defined(EIGEN_MPL2_ONLY) || (EIGEN_WORLD_VERSION > 3) || (EIGEN_MAJOR_VERSION > 3)
        template <int t_Options, typename t_StorageIndex>
        static void factorizeCholeskyInPlace(Eigen::SparseMatrix<t_Scalar, t_Options, t_StorageIndex> &H)
        {
            const Eigen::SimplicialLLT<typename std::decay<decltype(H)>::type, Eigen::Lower> llt(H);
            QPMAD_UTILS_PERSISTENT_ASSERT(
                    Eigen::Success == llt.info(), "Could not perform Cholesky decomposition of the Hessian (sparse).");
            H = llt.matrixL();
        }
#endif


        template <class t_primal>
        void setZero(t_primal &primal)
        {
            primal.setZero(primal_size_);
        }

        static void setZero(VectorRef primal)
        {
            primal.setZero();
        }
    };
}  // namespace qpmad
