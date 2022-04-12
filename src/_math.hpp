#ifndef __MATH_HPP
#define __MATH_HPP

#include <iostream>

namespace Math {
    template<class T>
    inline T limit(T v, T min, T max) {
        if(min > max)
            std::cerr << "function{limit} error : min > max" << std::endl;
        return v > max ? max : (v < min ? min : v);
    }

    inline float scale(float v, float from_min, float from_max, float to_min, float to_max) {
        if(from_min > from_max)
            std::cerr << "function{limit} error : from_min > from_max" << std::endl;
        if(to_min > to_max)
            std::cerr << "function{limit} error : to_min > to_max" << std::endl;
        return ((v - from_min) / (from_max - from_min)) * (to_max - to_min) + to_min;
    }

    inline double scale(double v, double from_min, double from_max, double to_min, double to_max) {
        if(from_min > from_max)
            std::cerr << "function{limit} error : from_min > from_max" << std::endl;
        if(to_min > to_max)
            std::cerr << "function{limit} error : to_min > to_max" << std::endl;
        return ((v - from_min) / (from_max - from_min)) * (to_max - to_min) + to_min;
    }
}

#endif