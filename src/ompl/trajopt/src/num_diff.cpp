#include "ompl/trajopt/num_diff.hpp"
using namespace Eigen;
using namespace sco;

namespace sco {

ScalarOfVectorPtr ScalarOfVector::construct(const boost_func& f) {
  struct F : public ScalarOfVector {
    boost_func f;
    F(const boost_func& _f) : f(_f) {}
    double operator()(const VectorXd& x) const {
      return f(x);
    }
  };
  ScalarOfVector* sov = new F(f); // to avoid erroneous clang warning
  return ScalarOfVectorPtr(sov);
}
VectorOfVectorPtr VectorOfVector::construct(const boost_func& f) {
  struct F : public VectorOfVector {
    boost_func f;
    F(const boost_func& _f) : f(_f) {}
    VectorXd operator()(const VectorXd& x) const {
      return f(x);
    }
  };
  VectorOfVector* vov = new F(f); // to avoid erroneous clang warning
  return VectorOfVectorPtr(vov);
}

VectorXd calcForwardNumGrad(const ScalarOfVector& f, const VectorXd& x, double epsilon) {
  VectorXd out(x.size());
  VectorXd xpert = x;
  double y = f(x);
  for (size_t i=0; i < size_t(x.size()); ++i) {
    xpert(i) = x(i) + epsilon;
    double ypert = f(xpert);
    out(i) = (ypert - y) / epsilon;
    xpert(i) = x(i);
  }
  return out;
}
MatrixXd calcForwardNumJac(const VectorOfVector& f, const VectorXd& x, double epsilon) {
  VectorXd y = f(x);
  MatrixXd out(y.size(), x.size());
  VectorXd xpert = x;
  for (size_t i=0; i < size_t(x.size()); ++i) {
    xpert(i) = x(i) + epsilon;
    VectorXd ypert = f(xpert);
    out.col(i) = (ypert - y) / epsilon;
    xpert(i) = x(i);
  }
  return out;
}

void calcGradAndDiagHess(const ScalarOfVector& f, const VectorXd& x,
    double epsilon, double& y, VectorXd& grad, VectorXd& hess) {
  y = f(x);
  grad.resize(x.size());
  hess.resize(x.size());
  VectorXd xpert = x;
  for (size_t i=0; i < size_t(x.size()); ++i) {
    xpert(i) = x(i) + epsilon/2;
    double yplus = f(xpert);
    xpert(i) = x(i) - epsilon/2;
    double yminus = f(xpert);
    grad(i) = (yplus - yminus)/epsilon;
    hess(i) = (yplus + yminus - 2*y) / (epsilon*epsilon/4);
    xpert(i) = x(i);
  }
}

void calcGradHess(ScalarOfVectorPtr f, const VectorXd& x, double epsilon,
    double& y, VectorXd& grad, MatrixXd& hess) {
  y = f->call(x);
  VectorOfVectorPtr grad_func = forwardNumGrad(f, epsilon);
  grad = grad_func->call(x);
  hess = calcForwardNumJac(*grad_func, x, epsilon);
  hess = (hess + hess.transpose())/2;
}


struct ForwardNumGrad : public VectorOfVector {
  ScalarOfVectorPtr f_;
  double epsilon_;
  ForwardNumGrad(ScalarOfVectorPtr f, double epsilon) : f_(f), epsilon_(epsilon) {}
  VectorXd operator()(const VectorXd& x) const {
    return calcForwardNumGrad(*f_, x, epsilon_);
  }
};

struct ForwardNumJac : public MatrixOfVector {
  VectorOfVectorPtr f_;
  double epsilon_;
  ForwardNumJac(VectorOfVectorPtr f, double epsilon) : f_(f), epsilon_(epsilon) {}
  MatrixXd operator()(const VectorXd& x) const {
    return calcForwardNumJac(*f_, x, epsilon_);
  }
};

VectorOfVectorPtr forwardNumGrad(ScalarOfVectorPtr f, double epsilon) {
  return VectorOfVectorPtr(new ForwardNumGrad(f, epsilon));
}
MatrixOfVectorPtr forwardNumJac(VectorOfVectorPtr f, double epsilon) {
  return MatrixOfVectorPtr(new ForwardNumJac(f, epsilon));
}


}
