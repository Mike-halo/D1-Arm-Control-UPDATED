// d1_fk.hpp — Forward kinematics for the Unitree D1-550 arm (standard DH).
//
// Same math as the hand calculation:
//   1. convert joint angles q (from the arm) into DH angles theta
//   2. build one 4x4 matrix A_i per joint
//   3. multiply T = A1*A2*A3*A4*A5*A6*T_tool
//   4. read the position from T's last column
//
// No SDK dependency — include it anywhere.
#pragma once

#include <array>
#include <cmath>

namespace d1 {

using Mat4 = std::array<std::array<double, 4>, 4>;

constexpr double PI = 3.14159265358979323846;
constexpr double DEG2RAD = PI / 180.0;

// ---------------------------------------------------------------- DH table
// Row i:  theta_i = sign_i * q_i + offset_i   (theta is what goes into cos/sin)
//         d_i, a_i, alpha_i are fixed geometry (meters, radians).
struct DHRow {
    double sign;    // +1 or -1: does the arm count this joint the same way DH does?
    double offset;  // added to theta (radians)
    double d;       // slide along old z
    double a;       // slide along new x (link length)
    double alpha;   // twist about new x
};

// Derived from the D1-550 URDF. If a joint turns the opposite way on the
// real arm, flip its sign here — nothing else needs to change.
constexpr std::array<DHRow, 6> DH_TABLE = {{
    //  sign   offset      d        a        alpha
    {  -1.0,  0.0,       0.1316,  0.0,     -PI / 2 },   // J1 base yaw
    {  +1.0, -PI / 2,    0.0,     0.270,    0.0    },   // J2 shoulder pitch
    {  +1.0,  0.0,       0.0,     0.0413,  -PI / 2 },   // J3 elbow pitch
    {  +1.0,  0.0,       0.2047,  0.0,      PI / 2 },   // J4 forearm roll
    {  +1.0,  0.0,       0.0,     0.0,     -PI / 2 },   // J5 wrist pitch
    {  +1.0,  0.0,       0.0777,  0.0,      0.0    },   // J6 wrist roll -> flange
}};

// Distance from the flange (end of J6) to your gripper point, along the
// gripper's pointing direction. Measure this on the real arm.
constexpr double TOOL_D = 0.0;

// ---------------------------------------------------------------- matrices
inline Mat4 identity() {
    Mat4 I{};
    for (int i = 0; i < 4; ++i) I[i][i] = 1.0;
    return I;
}

// Row-times-column, exactly like the hand calculation.
inline Mat4 multiply(const Mat4& A, const Mat4& B) {
    Mat4 C{};
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            for (int k = 0; k < 4; ++k)
                C[r][c] += A[r][k] * B[k][c];
    return C;
}

// The general DH matrix: Rot_z(theta) * Trans_z(d) * Trans_x(a) * Rot_x(alpha)
inline Mat4 dh(double theta, double d, double a, double alpha) {
    const double c = std::cos(theta), s = std::sin(theta);
    const double ca = std::cos(alpha), sa = std::sin(alpha);
    return {{
        { c, -s * ca,  s * sa, a * c },
        { s,  c * ca, -c * sa, a * s },
        { 0,  sa,      ca,     d     },
        { 0,  0,       0,      1     },
    }};
}

// ---------------------------------------------------------------- FK
// q_deg: the six joint angles from the arm, in degrees.
// Returns the full 4x4 pose of the gripper point in the base frame.
inline Mat4 forward_kinematics(const std::array<double, 6>& q_deg) {
    Mat4 T = identity();
    for (int i = 0; i < 6; ++i) {
        const DHRow& row = DH_TABLE[i];
        const double theta = row.sign * q_deg[i] * DEG2RAD + row.offset;  // step 1
        T = multiply(T, dh(theta, row.d, row.a, row.alpha));              // steps 2-3
    }
    return multiply(T, dh(0.0, TOOL_D, 0.0, 0.0));  // Trans_z(TOOL_D)
}

struct Pose {
    double x, y, z;           // position, meters
    double roll, pitch, yaw;  // orientation, degrees (ZYX convention)
};

// Step 4: read position from the last column, and orientation from the 3x3.
inline Pose to_pose(const Mat4& T) {
    Pose p{};
    p.x = T[0][3];
    p.y = T[1][3];
    p.z = T[2][3];
    p.pitch = std::asin(-std::fmax(-1.0, std::fmin(1.0, T[2][0]))) / DEG2RAD;
    p.roll  = std::atan2(T[2][1], T[2][2]) / DEG2RAD;
    p.yaw   = std::atan2(T[1][0], T[0][0]) / DEG2RAD;
    return p;
}

}  // namespace d1