#pragma once

#include <Eigen/Core>
#include <vector>

namespace sco
{
    /** \brief Converts an eigen vector to a std::vector<double>. */
    inline std::vector<double> toDblVec(const Eigen::VectorXd &x)
    {
        return std::vector<double>(x.data(), x.data() + x.size());
    }

    /** \brief Converts a std::vector<double> to an eigen vector. */
    inline Eigen::VectorXd toVectorXd(const std::vector<double> &x)
    {
        return Eigen::Map<const Eigen::VectorXd>(x.data(), x.size());
    }
}
