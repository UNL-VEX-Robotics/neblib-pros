#pragma once

namespace neblib
{
    /// @brief Compute the mathematical sign of a number.
    ///
    /// Returns `1` if the input is positive, `-1` if negative, and `0` if zero.
    /// Works with any numeric type that supports comparisons with zero.
    ///
    /// @tparam T Numeric type of the input value (e.g., int, float, double).
    /// @param num The number whose sign is to be determined.
    /// @return `1` if `num > 0`, `-1` if `num < 0`, or `0` if `num == 0`.
    template <typename T>
    inline int sign(T num)
    {
        return (T(0) < num) - (num < T(0));
    }

    double wrap(double num, const double min, const double max);
}