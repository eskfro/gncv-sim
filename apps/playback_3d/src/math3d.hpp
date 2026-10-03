#pragma once
// ============================================================================
// Small 3D math for the renderer: vectors and 4x4 matrices in double
// precision, converted to float only when sent to OpenGL.
//
// World coordinates are NED (x north, y east, z down) and body coordinates
// are (x forward, y starboard, z down), both right handed, so the usual
// OpenGL matrix formulas apply unchanged.
// ============================================================================
#include <array>
#include <cmath>

namespace playback3d {

struct V3 {
    double x{};
    double y{};
    double z{};
};

inline V3 operator+(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline V3 operator-(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline V3 operator*(V3 a, double k) { return {a.x * k, a.y * k, a.z * k}; }
inline V3 operator*(double k, V3 a) { return a * k; }
inline double Dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3 Cross(V3 a, V3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline double Length(V3 a) { return std::sqrt(Dot(a, a)); }
inline V3 Normalize(V3 a) {
    const double n = Length(a);
    return n > 0.0 ? a * (1.0 / n) : V3{};
}

// Column major (OpenGL layout): m[col * 4 + row]
struct Mat4 {
    std::array<double, 16> m{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

    double& operator()(int row, int col) { return m[col * 4 + row]; }
    double operator()(int row, int col) const { return m[col * 4 + row]; }

    std::array<float, 16> ToFloat() const {
        std::array<float, 16> f{};
        for (int i = 0; i < 16; i++) f[i] = static_cast<float>(m[i]);
        return f;
    }
};

inline Mat4 operator*(const Mat4& a, const Mat4& b) {
    Mat4 r;
    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            double sum = 0.0;
            for (int k = 0; k < 4; k++) sum += a(row, k) * b(k, col);
            r(row, col) = sum;
        }
    }
    return r;
}

inline V3 TransformPoint(const Mat4& a, V3 p) {
    return {a(0, 0) * p.x + a(0, 1) * p.y + a(0, 2) * p.z + a(0, 3),
            a(1, 0) * p.x + a(1, 1) * p.y + a(1, 2) * p.z + a(1, 3),
            a(2, 0) * p.x + a(2, 1) * p.y + a(2, 2) * p.z + a(2, 3)};
}

// Homogeneous result of a * (p, 1)
inline std::array<double, 4> TransformClip(const Mat4& a, V3 p) {
    std::array<double, 4> r{};
    for (int row = 0; row < 4; row++) r[row] = a(row, 0) * p.x + a(row, 1) * p.y + a(row, 2) * p.z + a(row, 3);
    return r;
}

inline Mat4 Translate(V3 t) {
    Mat4 r;
    r(0, 3) = t.x;
    r(1, 3) = t.y;
    r(2, 3) = t.z;
    return r;
}

inline Mat4 Scale(double k) {
    Mat4 r;
    r(0, 0) = r(1, 1) = r(2, 2) = k;
    return r;
}

// Rotation about z by angle (positive turns x towards y)
inline Mat4 RotZ(double a) {
    Mat4 r;
    r(0, 0) = std::cos(a);
    r(0, 1) = -std::sin(a);
    r(1, 0) = std::sin(a);
    r(1, 1) = std::cos(a);
    return r;
}

// Body -> NED rotation R_zyx(phi, theta, psi) = Rz(psi) Ry(theta) Rx(phi),
// same as common::R_zyx in src/common.hpp
inline Mat4 RotZYX(double phi, double theta, double psi) {
    const double cf = std::cos(phi), sf = std::sin(phi);
    const double ct = std::cos(theta), st = std::sin(theta);
    const double cp = std::cos(psi), sp = std::sin(psi);
    Mat4 r;
    r(0, 0) = cp * ct;
    r(0, 1) = -sp * cf + cp * st * sf;
    r(0, 2) = sp * sf + cp * cf * st;
    r(1, 0) = sp * ct;
    r(1, 1) = cp * cf + sf * st * sp;
    r(1, 2) = -cp * sf + st * sp * cf;
    r(2, 0) = -st;
    r(2, 1) = ct * sf;
    r(2, 2) = ct * cf;
    return r;
}

// Right handed perspective, clip z in [-1, 1]
inline Mat4 Perspective(double fov_y, double aspect, double near_z, double far_z) {
    const double f = 1.0 / std::tan(0.5 * fov_y);
    Mat4 r;
    r(0, 0) = f / aspect;
    r(1, 1) = f;
    r(2, 2) = (far_z + near_z) / (near_z - far_z);
    r(2, 3) = 2.0 * far_z * near_z / (near_z - far_z);
    r(3, 2) = -1.0;
    r(3, 3) = 0.0;
    return r;
}

inline Mat4 LookAt(V3 eye, V3 target, V3 up) {
    const V3 f = Normalize(target - eye);
    const V3 s = Normalize(Cross(f, up));
    const V3 u = Cross(s, f);
    Mat4 r;
    r(0, 0) = s.x;
    r(0, 1) = s.y;
    r(0, 2) = s.z;
    r(1, 0) = u.x;
    r(1, 1) = u.y;
    r(1, 2) = u.z;
    r(2, 0) = -f.x;
    r(2, 1) = -f.y;
    r(2, 2) = -f.z;
    r(0, 3) = -Dot(s, eye);
    r(1, 3) = -Dot(u, eye);
    r(2, 3) = Dot(f, eye);
    return r;
}

}  // namespace playback3d
