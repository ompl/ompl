#pragma once

#include <Eigen/Core>
#include <vector>

namespace util {

inline std::vector<double> toDblVec(const Eigen::VectorXd& x) {
  return std::vector<double>(x.data(), x.data()+x.size());
}
inline Eigen::VectorXd toVectorXd(const std::vector<double>& x) {
  return Eigen::Map<const Eigen::VectorXd>(x.data(), x.size());
}

}

