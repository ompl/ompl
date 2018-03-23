#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

namespace sco
{
    inline double vecSum(const std::vector<double> &v)
    {
        double out = 0;
        for (size_t i = 0; i < v.size(); ++i)
            out += v[i];
        return out;
    }

    inline double vecAbsSum(const std::vector<double> &v)
    {
        double out = 0;
        for (size_t i = 0; i < v.size(); ++i)
            out += fabs(v[i]);
        return out;
    }

    inline double pospart(double x)
    {
        return (x > 0) ? x : 0;
    }

    inline double sq(double x)
    {
        return x * x;
    }

    inline double vecHingeSum(const std::vector<double> &v)
    {
        double out = 0;
        for (size_t i = 0; i < v.size(); ++i)
            out += pospart(v[i]);
        return out;
    }

    inline double vecMax(const std::vector<double> &v)
    {
        return *std::max_element(v.begin(), v.end());
    }

    inline double vecDot(const std::vector<double> &a, const std::vector<double> &b)
    {
        assert(a.size() == b.size());
        double out = 0;
        for (size_t i = 0; i < a.size(); ++i)
            out += a[i] * b[i];
        return out;
    }
}  // namespace sco
