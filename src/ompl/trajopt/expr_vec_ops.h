#include <Eigen/Core>
#include "solver_interface.h"

namespace sco
{
    AffExpr varDot(const Eigen::VectorXd &x, const VarVector &v)
    {
        AffExpr out;
        out.constant = 0;
        out.vars = v;
        out.coeffs = std::vector<double>(x.data(), x.data() + x.size());
        return out;
    }

}  // namespace sco
