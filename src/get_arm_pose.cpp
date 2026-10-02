// get_arm_pose.cpp — reads live joint angles from the D1 (same topic as
// get_arm_joint_angle.cpp) and prints the estimated gripper position.
//
// Put this file in d1_sdk/src/examples/ and d1_fk.hpp next to it.

#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/common/time/time_tool.hpp>
#include "msg/PubServoInfo_.hpp"
#include <array>
#include <chrono>
#include <cstdio>
#include "d1_fk.hpp"

#define TOPIC "current_servo_angle"

using namespace unitree::robot;
using namespace unitree::common;

// The arm publishes fast; print at most this often so the terminal is readable.
constexpr auto PRINT_PERIOD = std::chrono::milliseconds(200);

void Handler(const void* msg)
{
    static auto last_print = std::chrono::steady_clock::time_point{};
    const auto now = std::chrono::steady_clock::now();
    if (now - last_print < PRINT_PERIOD) return;
    last_print = now;

    const auto* pm = (const unitree_arm::msg::dds_::PubServoInfo_*)msg;

    // servo0..servo5 = joints 1..6 (degrees). servo6 is the gripper, not used by FK.
    const std::array<double, 6> q = {
        pm->servo0_data_(), pm->servo1_data_(), pm->servo2_data_(),
        pm->servo3_data_(), pm->servo4_data_(), pm->servo5_data_(),
    };

    const d1::Pose p = d1::to_pose(d1::forward_kinematics(q));

    std::printf("q [deg]: %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f  |  "
                "pos [m]: x=%.4f y=%.4f z=%.4f  |  rpy [deg]: %6.1f %6.1f %6.1f\n",
                q[0], q[1], q[2], q[3], q[4], q[5],
                p.x, p.y, p.z, p.roll, p.pitch, p.yaw);
    std::fflush(stdout);
}

int main()
{
    ChannelFactory::Instance()->Init(0);
    ChannelSubscriber<unitree_arm::msg::dds_::PubServoInfo_> subscriber(TOPIC);
    subscriber.InitChannel(Handler);

    std::printf("Listening on '%s' — move the arm and watch the position.\n", TOPIC);
    while (true) sleep(10);
    return 0;
}