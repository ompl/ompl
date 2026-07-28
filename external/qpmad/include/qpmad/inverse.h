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
    class TriangularInversion
    {
    public:
        template <class t_OutputMatrixType, class t_InputMatrixType>
        static void compute(t_OutputMatrixType &U_inverse, const t_InputMatrixType &L)
        {
            const MatrixIndex size = L.rows();

            for (MatrixIndex i = 0; i < size; ++i)
            {
                U_inverse(i, i) = 1.0 / L(i, i);
                for (MatrixIndex j = i - 1; j >= 0; --j)
                {
                    const double tmp =
                            L.transpose().row(j).segment(j + 1, i - j) * U_inverse.col(i).segment(j + 1, i - j);
                    U_inverse(j, i) = -tmp / L(j, j);
                }
            }
        }
    };
}  // namespace qpmad
