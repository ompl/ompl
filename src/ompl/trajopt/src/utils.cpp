#include <Eigen/Geometry>
#include <boost/format.hpp>

#include "ompl/trajopt/solver_interface.h"
#include "ompl/trajopt/utils.h"

using namespace Eigen;

namespace sco
{
    TrajArray getTraj(const DblVec &x, const VarArray &vars)
    {
        TrajArray out(vars.rows(), vars.cols());
        for (int i = 0; i < vars.rows(); ++i)
        {
            for (int j = 0; j < vars.cols(); ++j)
            {
                out(i, j) = vars(i, j).value(x);
            }
        }
        return out;
    }

    TrajArray getTraj(const DblVec &x, const AffArray &arr)
    {
        MatrixXd out(arr.rows(), arr.cols());
        for (int i = 0; i < arr.rows(); ++i)
        {
            for (int j = 0; j < arr.cols(); ++j)
            {
                out(i, j) = arr(i, j).value(x);
            }
        }
        return out;
    }

    void AddVarArrays(sco::OptProb &prob, int rows, const vector<int> &cols, const vector<std::string> &name_prefix,
                      const vector<VarArray *> &newvars)
    {
        int n_arr = name_prefix.size();
        assert(n_arr == newvars.size());

        vector<MatrixXi> index(n_arr);
        for (int i = 0; i < n_arr; ++i)
        {
            newvars[i]->resize(rows, cols[i]);
            index[i].resize(rows, cols[i]);
        }

        vector<std::string> names;
        int var_idx = prob.getNumVars();
        for (int i = 0; i < rows; ++i)
        {
            for (int k = 0; k < n_arr; ++k)
            {
                for (int j = 0; j < cols[k]; ++j)
                {
                    index[k](i, j) = var_idx;
                    names.push_back((boost::format("%s_%i_%i") % name_prefix[k] % i % j).str());
                    ++var_idx;
                }
            }
        }
        prob.createVariables(names);  // note that w,r, are both unbounded

        const vector<sco::Var> &vars = prob.getVars();
        for (int k = 0; k < n_arr; ++k)
        {
            for (int i = 0; i < rows; ++i)
            {
                for (int j = 0; j < cols[k]; ++j)
                {
                    (*newvars[k])(i, j) = vars[index[k](i, j)];
                }
            }
        }
    }

    void AddVarArray(sco::OptProb &prob, int rows, int cols, const std::string &name_prefix, VarArray &newvars)
    {
        vector<VarArray *> arrs(1, &newvars);
        vector<std::string> prefixes(1, name_prefix);
        vector<int> colss(1, cols);
        AddVarArrays(prob, rows, colss, prefixes, arrs);
    }
}
