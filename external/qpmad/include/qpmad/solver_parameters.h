/**
    @file
    @author  Alexander Sherikov

    @copyright 2017 Alexander Sherikov. Licensed under the Apache License,
    Version 2.0. (see LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace qpmad
{
    class SolverParameters
    {
    public:
        enum HessianType
        {
            UNDEFINED = 0,
            HESSIAN_LOWER_TRIANGULAR = 1,
            HESSIAN_CHOLESKY_FACTOR = 2,
            HESSIAN_INVERTED_CHOLESKY_FACTOR = 3
            // HESSIAN_DIAGONAL         = 1,
        };


    public:
        HessianType hessian_type_;

        double tolerance_;

        std::ptrdiff_t max_iter_;

        bool return_inverted_cholesky_factor_;


    public:
        SolverParameters()
        {
            // default hessian type
            hessian_type_ = HESSIAN_LOWER_TRIANGULAR;

            tolerance_ = 1e-12;

            // -1 -> unlimited
            max_iter_ = -1;

            // this operation requires an extra copy and is not needed for
            // problems with varying Hessian, but allows skipping Hessian
            // inversion otherwise.
            return_inverted_cholesky_factor_ = false;
        }
    };
}  // namespace qpmad
