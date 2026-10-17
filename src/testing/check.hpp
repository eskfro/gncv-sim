#pragma once
// ============================================================================
// Minimal test helpers shared by the unit tests (no framework needed).
//
//   CHECK(cond);                 // records a failure and keeps going
//   return testing::Summary();   // from main(): exit code 0 when all passed
// ============================================================================
#include <cmath>
#include <cstdio>

namespace testing {

inline int& Failures() {
    static int failures = 0;
    return failures;
}

inline bool AlmostEqual(double a, double b, double tol = 1e-9) { return std::abs(a - b) <= tol; }

inline int Summary() {
    if (Failures() > 0) std::fprintf(stderr, "%d check(s) failed\n", Failures());
    return Failures() == 0 ? 0 : 1;
}

}  // namespace testing

#define CHECK(cond)                                                                       \
    do {                                                                                  \
        if (!(cond)) {                                                                    \
            std::fprintf(stderr, "%s:%d: CHECK(%s) failed\n", __FILE__, __LINE__, #cond); \
            testing::Failures()++;                                                        \
        }                                                                                 \
    } while (0)
