#include "util.hpp"

double neblib::wrap(double num, const double min, const double max)
{
    while (num < min)
        num += max - min;
    while (num > max)
        num -= max - min;
    return num;
}