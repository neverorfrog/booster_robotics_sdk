#ifndef __BOOSTER_ROBOTICS_SDK_ROBOT_STATE_HPP__
#define __BOOSTER_ROBOTICS_SDK_ROBOT_STATE_HPP__

namespace booster {
namespace robot {

/*Robot mode */
enum class RobotMode {
    kUnknown = -1, // For error handling
    kDamping = 0,  // All motor enter damping mode, robot will fall down if it is not supported
    kPrepare = 1,  // Prepare mode, the robot keeps standing on both feet and can switch to walking mode
    kWalking = 2,  // Walking mode, in walking mode, the robot can move, rotate, kick the ball, etc.
    kCustom = 3,   // Custom mode, in custom mode, the robot can do some custom actions
};

}
} // namespace booster::robot

#endif