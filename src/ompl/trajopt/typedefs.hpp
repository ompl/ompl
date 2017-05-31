#pragma once
#include <vector>
#include <map>
#include <Eigen/Core>
#include <boost/shared_ptr.hpp>

#include "ompl/trajopt/modeling.hpp"
#include "ompl/trajopt/basic_array.hpp"
#include "macros.h"

namespace trajopt {

using std::vector;
using std::map;

typedef util::BasicArray<sco::Var> VarArray;
typedef util::BasicArray<sco::AffExpr> AffArray;
typedef util::BasicArray<sco::Cnt> CntArray;

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> DblMatrix;

typedef vector<double> DblVec;
typedef vector<int> IntVec;

using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> TrajArray;
using Eigen::MatrixXd;
using Eigen::Matrix3d;

}
