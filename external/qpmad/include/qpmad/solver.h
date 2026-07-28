/**
    @file
    @author  Alexander Sherikov

    @copyright 2017 Alexander Sherikov. Licensed under the Apache License,
    Version 2.0. (see LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief Solver API layer
*/


#pragma once

#include "implementation.h"


namespace qpmad
{
    template <typename t_Scalar, int... t_Parameters>
    class SolverTemplate : public SolverBase<t_Scalar, t_Parameters...>
    {
    public:
        using typename SolverBase<t_Scalar, t_Parameters...>::ReturnStatus;


    protected:
        template <typename t_Type>
        using VectorEnablerReturnType =
                const typename std::enable_if<t_Type::IsVectorAtCompileTime, ReturnStatus>::type;
        template <typename t_Type>
        using MatrixEnablerReturnType =
                const typename std::enable_if<t_Type::ColsAtCompileTime != 1, ReturnStatus>::type;

        using typename SolverBase<t_Scalar, t_Parameters...>::MatrixRef;
        using typename SolverBase<t_Scalar, t_Parameters...>::VectorRef;
        using MatrixConstRef = const Eigen::Ref<const Eigen::Matrix<t_Scalar, Eigen::Dynamic, Eigen::Dynamic>>;
        using VectorConstRef = const Eigen::Ref<const Eigen::Matrix<t_Scalar, Eigen::Dynamic, 1>>;


        const struct InputPlaceholders
        {
            Eigen::Matrix<t_Scalar, Eigen::Dynamic, 1> empty_vector_;
            Eigen::Matrix<t_Scalar, Eigen::Dynamic, Eigen::Dynamic> empty_matrix_;
            SolverParameters solver_parameters_;
        } input_placeholders_;


    protected:
        template <class t_primal, class... t_Args>
        VectorEnablerReturnType<t_primal> solve0(t_primal &primal, t_Args &&...args)
        {
            return (solve1(primal, std::forward<t_Args>(args)...));
        }

        template <class... t_Args>
        ReturnStatus solve0(VectorRef primal, t_Args &&...args)
        {
            return (solve1(primal, std::forward<t_Args>(args)...));
        }

        // ---

        template <class t_primal, class t_H, class... t_Args>
        MatrixEnablerReturnType<t_H> solve1(t_primal &primal, t_H &H, t_Args &&...args)
        {
            return (solve2(primal, H, std::forward<t_Args>(args)...));
        }

        template <class t_primal, class... t_Args>
        ReturnStatus solve1(t_primal &primal, MatrixRef H, t_Args &&...args)
        {
            return (solve2(primal, H, std::forward<t_Args>(args)...));
        }

        // ---

        template <class t_primal, class t_H, class t_h, class... t_Args>
        VectorEnablerReturnType<t_h> solve2(t_primal &primal, t_H &H, const t_h &h, t_Args &&...args)
        {
            return (solve3(primal, H, h, std::forward<t_Args>(args)...));
        }

        template <class t_primal, class t_H, class... t_Args>
        ReturnStatus solve2(t_primal &primal, t_H &H, VectorConstRef h, t_Args &&...args)
        {
            return (solve3(primal, H, h, std::forward<t_Args>(args)...));
        }

        // ---

        template <class t_primal, class t_H, class t_h, class t_lb, class... t_Args>
        VectorEnablerReturnType<t_lb> solve3(t_primal &primal, t_H &H, const t_h &h, const t_lb &lb, t_Args &&...args)
        {
            return (solve4(primal, H, h, lb, std::forward<t_Args>(args)...));
        }

        template <class t_primal, class t_H, class t_h, class t_lb, class... t_Args>
        ReturnStatus solve3(t_primal &primal, t_H &H, const t_h &h, VectorConstRef lb, t_Args &&...args)
        {
            return (solve4(primal, H, h, lb, std::forward<t_Args>(args)...));
        }

        template <class t_primal, class t_H, class t_h, class t_A, class... t_Args>
        MatrixEnablerReturnType<t_A> solve3(t_primal &primal, t_H &H, const t_h &h, const t_A &A, t_Args &&...args)
        {
            return (
                    solve5(primal,
                           H,
                           h,
                           input_placeholders_.empty_vector_,
                           input_placeholders_.empty_vector_,
                           A,
                           std::forward<t_Args>(args)...));
        }

        template <class t_primal, class t_H, class t_h, class... t_Args>
        ReturnStatus solve3(t_primal &primal, t_H &H, const t_h &h, MatrixConstRef A, t_Args &&...args)
        {
            return (
                    solve5(primal,
                           H,
                           h,
                           input_placeholders_.empty_vector_,
                           input_placeholders_.empty_vector_,
                           A,
                           std::forward<t_Args>(args)...));
        }

        // ---

        template <class t_primal, class t_H, class t_h, class t_lb, class t_ub>
        VectorEnablerReturnType<t_ub> solve4(t_primal &primal, t_H &H, const t_h &h, const t_lb &lb, const t_ub &ub)
        {
            return (solve5(primal, H, h, lb, ub, input_placeholders_.solver_parameters_));
        }


        template <class t_primal, class t_H, class t_h, class t_lb, class t_ub, class... t_Args>
        VectorEnablerReturnType<t_ub> solve4(
                t_primal &primal,
                t_H &H,
                const t_h &h,
                const t_lb &lb,
                const t_ub &ub,
                t_Args &&...args)
        {
            return (solve5(primal, H, h, lb, ub, std::forward<t_Args>(args)...));
        }

        template <class t_primal, class t_H, class t_h, class t_lb>
        ReturnStatus solve4(t_primal &primal, t_H &H, const t_h &h, const t_lb &lb, VectorConstRef ub)
        {
            return (solve5(primal, H, h, lb, ub, input_placeholders_.solver_parameters_));
        }


        template <class t_primal, class t_H, class t_h, class t_lb, class... t_Args>
        ReturnStatus solve4(t_primal &primal, t_H &H, const t_h &h, const t_lb &lb, VectorConstRef ub, t_Args &&...args)
        {
            return (solve5(primal, H, h, lb, ub, std::forward<t_Args>(args)...));
        }

        // ---

        template <class t_primal, class t_H, class t_h, class t_lb, class t_ub>
        ReturnStatus solve5(
                t_primal &primal,
                t_H &H,
                const t_h &h,
                const t_lb &lb,
                const t_ub &ub,
                const SolverParameters &param)
        {
            return (
                    solve8(primal,
                           H,
                           h,
                           lb,
                           ub,
                           input_placeholders_.empty_matrix_,
                           input_placeholders_.empty_vector_,
                           input_placeholders_.empty_vector_,
                           param));
        }

        template <class t_primal, class t_H, class t_h, class t_lb, class t_ub, class t_A, class... t_Args>
        MatrixEnablerReturnType<t_A> solve5(
                t_primal &primal,
                t_H &H,
                const t_h &h,
                const t_lb &lb,
                const t_ub &ub,
                const t_A &A,
                t_Args &&...args)
        {
            return (solve6(primal, H, h, lb, ub, A, std::forward<t_Args>(args)...));
        }

        template <class t_primal, class t_H, class t_h, class t_lb, class t_ub, class... t_Args>
        ReturnStatus solve5(
                t_primal &primal,
                t_H &H,
                const t_h &h,
                const t_lb &lb,
                const t_ub &ub,
                MatrixConstRef A,
                t_Args &&...args)
        {
            return (solve6(primal, H, h, lb, ub, A, std::forward<t_Args>(args)...));
        }

        // ---

        template <class t_primal, class t_H, class t_h, class t_lb, class t_ub, class t_A, class t_Alb>
        VectorEnablerReturnType<t_Alb> solve6(
                t_primal &primal,
                t_H &H,
                const t_h &h,
                const t_lb &lb,
                const t_ub &ub,
                const t_A &A,
                const t_Alb &Alb)
        {
            return (solve8(primal, H, h, lb, ub, A, Alb, Alb, input_placeholders_.solver_parameters_));
        }

        template <class t_primal, class t_H, class t_h, class t_lb, class t_ub, class t_A, class t_Alb, class... t_Args>
        VectorEnablerReturnType<t_Alb> solve6(
                t_primal &primal,
                t_H &H,
                const t_h &h,
                const t_lb &lb,
                const t_ub &ub,
                const t_A &A,
                const t_Alb &Alb,
                t_Args &&...args)
        {
            return (solve7(primal, H, h, lb, ub, A, Alb, std::forward<t_Args>(args)...));
        }

        template <class t_primal, class t_H, class t_h, class t_lb, class t_ub, class t_A>
        ReturnStatus solve6(
                t_primal &primal,
                t_H &H,
                const t_h &h,
                const t_lb &lb,
                const t_ub &ub,
                const t_A &A,
                VectorConstRef Alb)
        {
            return (solve8(primal, H, h, lb, ub, A, Alb, Alb, input_placeholders_.solver_parameters_));
        }

        template <class t_primal, class t_H, class t_h, class t_lb, class t_ub, class t_A, class... t_Args>
        ReturnStatus solve6(
                t_primal &primal,
                t_H &H,
                const t_h &h,
                const t_lb &lb,
                const t_ub &ub,
                const t_A &A,
                VectorConstRef Alb,
                t_Args &&...args)
        {
            return (solve7(primal, H, h, lb, ub, A, Alb, std::forward<t_Args>(args)...));
        }

        // ---

        template <class t_primal, class t_H, class t_h, class t_lb, class t_ub, class t_A, class t_Alb, class t_Aub>
        VectorEnablerReturnType<t_Aub> solve7(
                t_primal &primal,
                t_H &H,
                const t_h &h,
                const t_lb &lb,
                const t_ub &ub,
                const t_A &A,
                const t_Alb &Alb,
                const t_Aub &Aub)
        {
            return (solve8(primal, H, h, lb, ub, A, Alb, Aub, input_placeholders_.solver_parameters_));
        }

        template <
                class t_primal,
                class t_H,
                class t_h,
                class t_lb,
                class t_ub,
                class t_A,
                class t_Alb,
                class t_Aub,
                class... t_Args>
        VectorEnablerReturnType<t_Aub> solve7(
                t_primal &primal,
                t_H &H,
                const t_h &h,
                const t_lb &lb,
                const t_ub &ub,
                const t_A &A,
                const t_Alb &Alb,
                const t_Aub &Aub,
                t_Args &&...args)
        {
            return (solve8(primal, H, h, lb, ub, A, Alb, Aub, std::forward<t_Args>(args)...));
        }

        template <class t_primal, class t_H, class t_h, class t_lb, class t_ub, class t_A, class t_Alb>
        ReturnStatus solve7(
                t_primal &primal,
                t_H &H,
                const t_h &h,
                const t_lb &lb,
                const t_ub &ub,
                const t_A &A,
                const t_Alb &Alb,
                VectorConstRef Aub)
        {
            return (solve8(primal, H, h, lb, ub, A, Alb, Aub, input_placeholders_.solver_parameters_));
        }

        template <class t_primal, class t_H, class t_h, class t_lb, class t_ub, class t_A, class t_Alb, class... t_Args>
        ReturnStatus solve7(
                t_primal &primal,
                t_H &H,
                const t_h &h,
                const t_lb &lb,
                const t_ub &ub,
                const t_A &A,
                const t_Alb &Alb,
                VectorConstRef Aub,
                t_Args &&...args)
        {
            return (solve8(primal, H, h, lb, ub, A, Alb, Aub, std::forward<t_Args>(args)...));
        }

        // ---

        template <class t_primal, class t_H, class t_h, class t_lb, class t_ub, class t_A, class t_Alb, class t_Aub>
        ReturnStatus solve8(
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
            this->primal_size_ = H.rows();
            this->h_size_ = h.rows();

            QPMAD_UTILS_PERSISTENT_ASSERT(this->primal_size_ > 0, "Hessian must not be empty.");
            QPMAD_UTILS_PERSISTENT_ASSERT(this->primal_size_ == H.cols(), "Hessian must be square.");
            QPMAD_UTILS_PERSISTENT_ASSERT(
                    ((this->primal_size_ == this->h_size_) && (1 == h.cols())) || (0 == this->h_size_),
                    "Wrong size of h.");


            this->num_simple_bounds_ = lb.rows();

            QPMAD_UTILS_PERSISTENT_ASSERT(
                    (0 == this->num_simple_bounds_) || (this->primal_size_ == this->num_simple_bounds_),
                    "Vector of lower simple bounds has wrong size (1).");
            QPMAD_UTILS_PERSISTENT_ASSERT(
                    ub.rows() == this->num_simple_bounds_, "Vector of upper simple bounds has wrong size (1).");

            QPMAD_UTILS_PERSISTENT_ASSERT(
                    ((this->num_simple_bounds_ > 0) && (1 == lb.cols())) || (1 == lb.cols()),
                    "Vector of lower simple bounds has wrong size (2).");
            QPMAD_UTILS_PERSISTENT_ASSERT(
                    ((this->num_simple_bounds_ > 0) && (1 == ub.cols())) || (1 == ub.cols()),
                    "Vector of upper simple bounds has wrong size (2).");


            this->num_general_constraints_ = A.rows();

            QPMAD_UTILS_PERSISTENT_ASSERT(
                    (A.cols() == this->primal_size_) || ((0 == this->num_general_constraints_) && (0 == A.cols())),
                    "Matrix of general constraints has wrong size.");

            QPMAD_UTILS_PERSISTENT_ASSERT(
                    Alb.rows() == this->num_general_constraints_,
                    "Vector of lower bounds of general constraints has wrong size (1).");
            QPMAD_UTILS_PERSISTENT_ASSERT(
                    Aub.rows() == this->num_general_constraints_,
                    "Vector of upper bounds of general constraints has wrong size (1).");

            QPMAD_UTILS_PERSISTENT_ASSERT(
                    ((this->num_general_constraints_ > 0) && (1 == Alb.cols())) || (0 == Alb.rows()),
                    "Vector of lower bounds of general constraints has wrong size (2).");
            QPMAD_UTILS_PERSISTENT_ASSERT(
                    ((this->num_general_constraints_ > 0) && (1 == Aub.cols())) || (0 == Aub.rows()),
                    "Vector of upper bounds of general constraints has wrong size (2).");

            return (this->solveGeneric(primal, H, h, lb, ub, A, Alb, Aub, param));
        }


    public:
        SolverTemplate()
        {
        }

        /**
         * @brief Solve QP
         *
         * Inputs:
         *  - [out] Vector primal -- solution vector, mandatory, allocated if needed
         *  - [in,out] Matrix H -- Hessian, mandatory, non-empty, factorized in-place
         *  - [in] Vector h -- objective vector, mandatory, may be empty
         *  - [in] Vector lb -- vector of lower bounds, may be omitted or may be empty consistently with ub
         *  - [in] Vector ub -- vector of upper bounds, may be omitted or may be empty consistently with lb
         *  - [in] Matrix A -- general constraints matrix, may be omitted with all general constraints, may be empty
         *  - [in] Vector Alb -- lower bounds of general constraints, may be omitted with all general constraints, may
         * be empty
         *  - [in] Vector Aub -- upper bounds of general constraints, may be omitted (if A and Alb are present they
         * are assumed to be equality constraints), may be empty
         *  - [in] SolverParameters param -- solver parameters, may be omitted
         */
        template <class... t_Args>
        ReturnStatus solve(t_Args &&...args)
        {
            return (solve0(std::forward<t_Args>(args)...));
        }
    };


    using Solver = SolverTemplate<double, Eigen::Dynamic, 1, Eigen::Dynamic>;
}  // namespace qpmad
