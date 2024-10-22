#ifndef BOOSTER_ROBOTICS_SDK_B1_API_CONST_HPP
#define BOOSTER_ROBOTICS_SDK_B1_API_CONST_HPP

#include <string>

namespace booster {
namespace robot {
namespace b1 {

static const std::string kTopicJointCtrl = "rt/joint_ctrl";
static const std::string kTopicLowState = "rt/low_state";
static const std::string kTopicOdometerState = "rt/odometer_state";

// TODO(@wuyuanye): 按照结构图，把电机的索引完善
enum JointIndex {
    // head
    kHeadYaw = 0,
    kHeadPitch = 1,

    // Left arm
    kLeftShoulderPitch = 2,
    kLeftShoulderRoll = 3,
    kLeftElbowPitch = 4,
    kLeftElbowYaw = 5,

    // Right arm
    kRightShoulderPitch = 6,
    kRightShoulderRoll = 7,
    kRightElbowPitch = 8,
    kRightElbowYaw = 9,

    // waist
    kWaist = 10,

    // left leg
    kLeftHipPitch = 11,
    kLeftHipRoll = 12, 
    kLeftHipYaw = 13,
    kLeftKneePitch = 14,
    kCrankUpLeft = 15,
    kCrankDownLeft = 16,

    // right leg
    kRightHipPitch = 17,
    kRightHipRoll = 18,
    kRightHipYaw = 19,
    kRightKneePitch = 20,
    kCrankUpRight = 21,
    kCrankDownRight = 22,
};

static const size_t kJointCnt = 23;

enum HandIndex {
    kLeftHand = 0,
    kRightHand = 1,
};

enum HandAction {
    kHandOpen = 0,
    kHandClose = 1,
};

}
}
} // namespace booster::robot::b1

#endif // BOOSTER_ROBOTICS_SDK_B1_API_CONST_HPP