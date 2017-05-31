#pragma once
#include <boost/function.hpp>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>
/*
 * Numerical derivatives
 */

namespace sco {
using boost::function;
using Eigen::VectorXd;
using Eigen::MatrixXd;
class ScalarOfVector;
class VectorOfVector;
class MatrixOfVector;
typedef boost::shared_ptr<ScalarOfVector> ScalarOfVectorPtr;
typedef boost::shared_ptr<VectorOfVector> VectorOfVectorPtr;
typedef boost::shared_ptr<MatrixOfVector> MatrixOfVectorPtr;

class ScalarOfVector {
public:
  virtual double operator()(const VectorXd& x) const = 0;
  double call(const VectorXd& x) const {return operator()(x);}
  virtual ~ScalarOfVector() {}

  typedef function<double(VectorXd)> boost_func;
  static ScalarOfVectorPtr construct(const boost_func&);
};

class VectorOfVector {
public:
  virtual VectorXd operator()(const VectorXd& x) const = 0;
  VectorXd call(const VectorXd& x) const {return operator()(x);}
  virtual ~VectorOfVector() {}

  typedef function<VectorXd(VectorXd)> boost_func;
  static VectorOfVectorPtr construct(const boost_func&);
};

class MatrixOfVector {
public:
    virtual MatrixXd operator()(const VectorXd& x) const = 0;
    MatrixXd call(const VectorXd& x) const {return operator()(x);}
    virtual ~MatrixOfVector() {}

    typedef function<MatrixXd(VectorXd)> boost_func;
    static MatrixOfVectorPtr construct(const boost_func&);
};


VectorXd calcForwardNumGrad(const ScalarOfVector& f, const VectorXd& x, double epsilon);
MatrixXd calcForwardNumJac(const VectorOfVector& f, const VectorXd& x, double epsilon);
void calcGradAndDiagHess(const ScalarOfVector& f, const VectorXd& x, double epsilon,
    double& y, VectorXd& grad, VectorXd& hess);
void calcGradHess(ScalarOfVectorPtr f, const VectorXd& x, double epsilon,
    double& y, VectorXd& grad, MatrixXd& hess);
VectorOfVectorPtr forwardNumGrad(ScalarOfVectorPtr f, double epsilon);
MatrixOfVectorPtr forwardNumJac(VectorOfVectorPtr f, double epsilon);

}
