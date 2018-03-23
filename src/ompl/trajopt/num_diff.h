#pragma once
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>
#include <functional>
/*
 * Numerical derivatives
 */

namespace sco
{
    using Eigen::VectorXd;
    using Eigen::MatrixXd;
    class ScalarOfVector;
    class VectorOfVector;
    class MatrixOfVector;
    typedef boost::shared_ptr<ScalarOfVector> ScalarOfVectorPtr;
    typedef boost::shared_ptr<VectorOfVector> VectorOfVectorPtr;
    typedef boost::shared_ptr<MatrixOfVector> MatrixOfVectorPtr;

    /** \brief A function that takes a vector (eigen) and returns a scalar. */
    class ScalarOfVector
    {
    public:
        virtual double operator()(const VectorXd &x) const = 0;
        double call(const VectorXd &x) const
        {
            return operator()(x);
        }
        virtual ~ScalarOfVector()
        {
        }

        typedef std::function<double(VectorXd)> func_vector2double;
        static ScalarOfVectorPtr construct(const func_vector2double &);
    };

    /** \brief A function that takes a vector of any size and retuns a vector (can be different size.) */
    class VectorOfVector
    {
    public:
        virtual VectorXd operator()(const VectorXd &x) const = 0;
        VectorXd call(const VectorXd &x) const
        {
            return operator()(x);
        }
        virtual ~VectorOfVector()
        {
        }

        typedef std::function<VectorXd(VectorXd)> func_vector2vector;
        static VectorOfVectorPtr construct(const func_vector2vector &);
    };

    class MatrixOfVector
    {
    public:
        virtual MatrixXd operator()(const VectorXd &x) const = 0;
        MatrixXd call(const VectorXd &x) const
        {
            return operator()(x);
        }
        virtual ~MatrixOfVector()
        {
        }

        typedef std::function<MatrixXd(VectorXd)> func_vector2Matrix;
        static MatrixOfVectorPtr construct(const func_vector2Matrix &);
    };

    VectorXd calcForwardNumGrad(const ScalarOfVector &f, const VectorXd &x, double epsilon);
    MatrixXd calcForwardNumJac(const VectorOfVector &f, const VectorXd &x, double epsilon);
    void calcGradAndDiagHess(const ScalarOfVector &f, const VectorXd &x, double epsilon, double &y, VectorXd &grad,
                             VectorXd &hess);
    void calcGradHess(ScalarOfVectorPtr f, const VectorXd &x, double epsilon, double &y, VectorXd &grad,
                      MatrixXd &hess);
    VectorOfVectorPtr forwardNumGrad(ScalarOfVectorPtr f, double epsilon);
    MatrixOfVectorPtr forwardNumJac(VectorOfVectorPtr f, double epsilon);
}
