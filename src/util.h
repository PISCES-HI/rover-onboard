#ifndef UTIL_H
#define UTIL_H

inline double map(double val, double min_a, double max_a, double min_b, double max_b) {
    return ((val - min_a)/(max_a - min_a))*(max_b-min_b) + min_b;
}

#endif // UTIL_H
