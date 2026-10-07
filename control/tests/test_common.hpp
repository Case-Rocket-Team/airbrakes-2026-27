#ifndef CONTROL_TESTS_TEST_COMMON_HPP
#define CONTROL_TESTS_TEST_COMMON_HPP

#include "base/base.h"

#include <cmath>

inline bool approximately_equal(
    f32 actual,
    f32 expected,
    f32 tolerance
) {
    return std::fabs(actual - expected) <= tolerance;
}

#endif  // CONTROL_TESTS_TEST_COMMON_HPP