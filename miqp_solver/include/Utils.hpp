#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace miqp_solver {
    // Function to check if a given value is an integer

    inline bool isClose(double a, double b, double epsilon = 1e-6) {
        return std::abs(a - b) < epsilon;
    }


    inline bool isInteger(double value) {
        return isClose(value, std::round(value));   
    }
}