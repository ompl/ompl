/**
    @file
    @author  Alexander Sherikov

    @copyright 2017 Alexander Sherikov. Licensed under the Apache License,
    Version 2.0. (see LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/


#pragma once

#include "common.h"


namespace qpmad
{
    /**
     * @brief
     *
     * Represents Givens rotation
     *
     *  [  cos, sin ]
     *  [           ]
     *  [ -sin, cos ]
     *
     * for a given vector (a, b) is defined with
     *
     *  [  cos, sin ]       [ a ]       [ d ]
     *  [           ]   *   [   ]   =   [   ]
     *  [ -sin, cos ]       [ b ]       [ 0 ]
     *
     *  sin^2 + cos^2 = 1
     *
     * Special cases:
     *  COPY        b == 0: cos = 1, sin = 0
     *  SWAP    (b != 0) && (a == 0): cos = 0, sin = 1
     */
    template <typename t_Scalar>
    class GivensRotation
    {
    public:
        enum Type
        {
            NONTRIVIAL = 0,
            COPY = 1,
            SWAP = 2
        };


    public:
        GivensRotation() = default;

        Type computeAndApply(t_Scalar &a, t_Scalar &b, const t_Scalar eps)
        {
            t_Scalar abs_b = std::fabs(b);

            if (abs_b > eps)
            {
                t_Scalar abs_a = std::fabs(a);

                if (abs_a > eps)
                {
                    t_Scalar t;
                    if (abs_a > abs_b)
                    {
                        t = (abs_b / abs_a);
                        t = abs_a * std::sqrt(1.0 + t * t);
                    }
                    else
                    {
                        t = (abs_a / abs_b);
                        t = abs_b * std::sqrt(1.0 + t * t);
                    }
                    t = copysign(t, a);

                    cos = a / t;
                    sin = b / t;

                    a = t;
                    b = 0.0;

                    type = NONTRIVIAL;
                }
                else
                {
                    // cos = 0.0;
                    // sin = 1.0;
                    swap(a, b);
                    type = SWAP;
                }
            }
            else
            {
                // cos = 1.0;
                // sin = 0.0;
                type = COPY;
            }

            return (type);
        }


        void apply(t_Scalar &a, t_Scalar &b) const
        {
            switch (type)
            {
                case COPY:
                    return;
                case SWAP:
                    swap(a, b);
                    return;
                case NONTRIVIAL:
                    applyNonTrivial(a, b);
                    return;
            }
        }

        template <class t_MatrixType>
        void applyColumnWise(
                t_MatrixType &M,
                MatrixIndex start,
                MatrixIndex end,
                MatrixIndex column_1,
                MatrixIndex column_2) const
        {
            switch (type)
            {
                case COPY:
                    return;
                case SWAP:
                    M.col(column_1).segment(start, end - start).swap(M.col(column_2).segment(start, end - start));
                    return;
                case NONTRIVIAL:
                    M.middleRows(start, end - start)
                            .transpose()
                            .applyOnTheLeft(column_1, column_2, Eigen::JacobiRotation<t_Scalar>(cos, sin));
                    return;
            }
        }


        template <class t_MatrixType>
        void applyRowWise(t_MatrixType &M, MatrixIndex start, MatrixIndex end, MatrixIndex row_1, MatrixIndex row_2)
                const
        {
            switch (type)
            {
                case COPY:
                    return;
                case SWAP:
                    M.row(row_1).segment(start, end - start).swap(M.row(row_2).segment(start, end - start));
                    return;
                case NONTRIVIAL:
                    M.middleCols(start, end - start)
                            .applyOnTheLeft(row_1, row_2, Eigen::JacobiRotation<t_Scalar>(cos, sin));
                    return;
            }
        }

    private:
        Type type;
        t_Scalar cos;
        t_Scalar sin;


    private:
        inline void swap(t_Scalar &a, t_Scalar &b) const
        {
            std::swap(a, b);
        }

        inline void applyNonTrivial(t_Scalar &a, t_Scalar &b) const
        {
            t_Scalar t1 = a;
            t_Scalar t2 = b;
            a = t1 * cos + t2 * sin;
            b = -sin * t1 + cos * t2;
        }
    };
}  // namespace qpmad
