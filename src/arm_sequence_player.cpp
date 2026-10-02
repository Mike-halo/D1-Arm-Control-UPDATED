#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/common/time/time_tool.hpp>
#include "msg/ArmString_.hpp"

#include <array>
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

#define TOPIC "rt/arm_Command"
#define NET_INTERFACE "enp0s31f6"

using namespace unitree::robot;
using namespace unitree::common;

// ---------------------------------------------------------------------------
// Define your sequence of poses here.
// Each pose is 7 angles: {J0, J1, J2, J3, J4, J5, J6}
//   J0 base(deg)  J1 shoulder(deg)  J2 elbow(deg)  J3 forearm roll(deg)
//   J4 wrist pitch(deg)  J5 wrist roll(deg)  J6 gripper(mm, 0-65)
// Edit these freely, or add/remove poses from the list.
// ---------------------------------------------------------------------------
struct Pose {
    std::array<double, 7> angles;
    int hold_ms;   // how long to pause after reaching this pose, before the next
};

static const std::vector<Pose> kSequence = {
    // angles:  {J0,   J1,   J2,  J3,  J4,  J5,  J6}    hold (ms)
    {{0,     0,    0,   0,   0,   0,   0},   2000},   // zero / home
    {{0,   -30,   30,   0,  20,   0,  20},   2000},   // gentle reach forward
    {{45,  -30,   30,   0,  20,   0,  20},   2000},   // rotate base right
    {{-45, -30,   30,   0,  20,   0,  20},   2000},   // rotate base left
    {{0,     0,    0,   0,   0,   0,   0},   2000},   // back to zero
};

static std::string BuildPoseJson(const Pose& p, int seq)
{
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "{\"seq\":%d,\"address\":1,\"funcode\":2,"
        "\"data\":{\"mode\":1,"
        "\"angle0\":%g,\"angle1\":%g,\"angle2\":%g,\"angle3\":%g,"
        "\"angle4\":%g,\"angle5\":%g,\"angle6\":%g}}",
        seq,
        p.angles[0], p.angles[1], p.angles[2], p.angles[3],
        p.angles[4], p.angles[5], p.angles[6]);
    return std::string(buf);
}

int main()
{
    ChannelFactory::Instance()->Init(0, NET_INTERFACE);
    ChannelPublisher<unitree_arm::msg::dds_::ArmString_> publisher(TOPIC);
    publisher.InitChannel();

    std::printf("Playing %zu-pose sequence. Ctrl+C to abort at any point.\n",
                kSequence.size());

    int seq = 4;
    for (size_t i = 0; i < kSequence.size(); ++i) {
        const Pose& pose = kSequence[i];
        std::string json = BuildPoseJson(pose, seq++);

        unitree_arm::msg::dds_::ArmString_ msg{};
        msg.data_() = json;
        publisher.Write(msg);

        std::printf("[%zu/%zu] Sent: %s\n", i + 1, kSequence.size(), json.c_str());

        std::this_thread::sleep_for(std::chrono::milliseconds(pose.hold_ms));
    }

    std::printf("Sequence complete.\n");
    return 0;
}