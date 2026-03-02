#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/chrono.h>

#include "booster/robot/b1/b1_loco_client.hpp"
#include "booster/robot/b1/b1_api_const.hpp"
#include "booster/robot/b1/b1_loco_api.hpp"
#include "booster/idl/b1/ImuState.h"
#include "booster/idl/b1/LowState.h"
#include "booster/idl/b1/MotorState.h"
#include "booster/idl/b1/LowCmd.h"
#include "booster/idl/b1/MotorCmd.h"
#include "booster/idl/b1/Odometer.h"
#include "booster/robot/common/robot_shared.hpp"
#include "booster/robot/common/entities.hpp"
#include "booster/robot/channel/channel_factory.hpp"
#include "booster/idl/b1/HandReplyData.h"
#include "booster/idl/b1/HandReplyParam.h"
#include "booster/idl/b1/HandTouchData.h"
#include "booster/idl/b1/HandTouchParam.h"

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace nb = nanobind;
namespace robot = booster::robot;
using booster_interface::msg::LowState;
using booster_interface::msg::MotorState;
using booster_interface::msg::LowCmd;
using booster_interface::msg::MotorCmd;
using booster_interface::msg::Odometer;
using booster_interface::msg::ImuState;
using booster_interface::msg::CmdType;
using booster_interface::msg::HandReplyData;
using booster_interface::msg::HandReplyParam;

using booster_interface::msg::HandTouchData;
using booster_interface::msg::HandTouchParam;

namespace booster::robot::b1 {
class __attribute__((visibility("hidden"))) B1LowStateSubscriber : public std::enable_shared_from_this<B1LowStateSubscriber> {
public:
    B1LowStateSubscriber(const std::function<void(const LowState*)> &py_handler) :
        py_handler_(py_handler) {
    }

    void InitChannel() {
        nb::gil_scoped_release release;
        auto weak_this = std::weak_ptr<B1LowStateSubscriber>(shared_from_this());
        channel_ptr_ = booster::robot::ChannelFactory::Instance()->CreateRecvChannel<LowState>(channel_name_, [weak_this](const void *msg) {
            if (auto shared_this = weak_this.lock()) {
                {
                    nb::gil_scoped_acquire acquire;
                    const LowState *low_state_msg = static_cast<const LowState *>(msg);
                    shared_this->py_handler_(low_state_msg);
                }
            }
        });
    }

    void CloseChannel() {
        nb::gil_scoped_release release;
        if (channel_ptr_) {
            booster::robot::ChannelFactory::Instance()->CloseReader(channel_name_);
            channel_ptr_.reset();
        }
    }

    const std::string &GetChannelName() const {
        return channel_name_;
    }

private:
    ChannelPtr<booster_interface::msg::LowState> channel_ptr_;
    std::function<void(const LowState*)> py_handler_;
    const std::string channel_name_ = kTopicLowState;
};

class __attribute__((visibility("hidden"))) B1LowHandTouchDataScriber : public std::enable_shared_from_this<B1LowHandTouchDataScriber> {
public:
    B1LowHandTouchDataScriber(const std::function<void(const HandTouchData*)> &py_handler) :
        py_handler_(py_handler) {
    }

    void InitChannel() {
        nb::gil_scoped_release release;
        auto weak_this = std::weak_ptr<B1LowHandTouchDataScriber>(shared_from_this());
        channel_ptr_ = booster::robot::ChannelFactory::Instance()->CreateRecvChannel<HandTouchData>(channel_name_, [weak_this](const void *msg) {
            if (auto shared_this = weak_this.lock()) {
                {
                    nb::gil_scoped_acquire acquire;
                    const HandTouchData *hand_data = static_cast<const HandTouchData *>(msg);
                    shared_this->py_handler_(hand_data);
                }
            }
        });
    }

    void CloseChannel() {
        nb::gil_scoped_release release;
        if (channel_ptr_) {
            booster::robot::ChannelFactory::Instance()->CloseReader(channel_name_);
            channel_ptr_.reset();
        }
    }

    const std::string &GetChannelName() const {
        return channel_name_;
    }

private:
    ChannelPtr<booster_interface::msg::HandTouchData> channel_ptr_;
    std::function<void(const HandTouchData*)> py_handler_;
    const std::string channel_name_ = "rt/booster_hand_touch_data";
};

class __attribute__((visibility("hidden"))) B1LowHandDataScriber : public std::enable_shared_from_this<B1LowHandDataScriber> {
public:
    B1LowHandDataScriber(const std::function<void(const HandReplyData*)> &py_handler) :
        py_handler_(py_handler) {
    }

    void InitChannel() {
        nb::gil_scoped_release release;
        auto weak_this = std::weak_ptr<B1LowHandDataScriber>(shared_from_this());
        channel_ptr_ = booster::robot::ChannelFactory::Instance()->CreateRecvChannel<HandReplyData>(channel_name_, [weak_this](const void *msg) {
            if (auto shared_this = weak_this.lock()) {
                {
                    nb::gil_scoped_acquire acquire;
                    const HandReplyData *hand_data = static_cast<const HandReplyData *>(msg);
                    shared_this->py_handler_(hand_data);
                }
            }
        });
    }

    void CloseChannel() {
        nb::gil_scoped_release release;
        if (channel_ptr_) {
            booster::robot::ChannelFactory::Instance()->CloseReader(channel_name_);
            channel_ptr_.reset();
        }
    }

    const std::string &GetChannelName() const {
        return channel_name_;
    }

private:
    ChannelPtr<booster_interface::msg::HandReplyData> channel_ptr_;
    std::function<void(const HandReplyData*)> py_handler_;
    const std::string channel_name_ = "rt/booster_hand_data";
};

class __attribute__((visibility("hidden"))) B1LowCmdPublisher {
public:
    explicit B1LowCmdPublisher() :
        channel_name_(kTopicJointCtrl) {
    }

    void InitChannel() {
        nb::gil_scoped_release release;
        channel_ptr_ = ChannelFactory::Instance()->CreateSendChannel<LowCmd>(channel_name_);
    }

    bool Write(LowCmd *msg) {
        if (channel_ptr_) {
            return channel_ptr_->Write(msg);
        }
        return false;
    }

    void CloseChannel() {
        nb::gil_scoped_release release;
        if (channel_ptr_) {
            ChannelFactory::Instance()->CloseWriter(channel_name_);
            channel_ptr_.reset();
        }
    }

    const std::string &GetChannelName() const {
        return channel_name_;
    }

private:
    std::string channel_name_;
    ChannelPtr<LowCmd> channel_ptr_;
};

class __attribute__((visibility("hidden"))) B1OdometerStateSubscriber : public std::enable_shared_from_this<B1OdometerStateSubscriber> {
public:
    B1OdometerStateSubscriber(const std::function<void(const Odometer*)> &py_handler) :
        py_handler_(py_handler) {
    }

    void InitChannel() {
        nb::gil_scoped_release release;
        auto weak_this = std::weak_ptr<B1OdometerStateSubscriber>(shared_from_this());
        channel_ptr_ = booster::robot::ChannelFactory::Instance()->CreateRecvChannel<Odometer>(channel_name_, [weak_this](const void *msg) {
            if (auto shared_this = weak_this.lock()) {
                {
                    nb::gil_scoped_acquire acquire;
                    const Odometer *low_state_msg = static_cast<const Odometer *>(msg);
                    shared_this->py_handler_(low_state_msg);
                }
            }
        });
    }

    void CloseChannel() {
        nb::gil_scoped_release release;
        if (channel_ptr_) {
            booster::robot::ChannelFactory::Instance()->CloseReader(channel_name_);
            channel_ptr_.reset();
        }
    }

    const std::string &GetChannelName() const {
        return channel_name_;
    }

private:
    ChannelPtr<Odometer> channel_ptr_;
    std::function<void(const Odometer*)> py_handler_;
    const std::string channel_name_ = kTopicOdometerState;
};
} // namespace booster::robot::b1

NB_MODULE(booster_robotics_sdk_python, m) {
    m.doc() = "Python binding of booster robotics sdk";

    nb::class_<robot::ChannelFactory>(m, "ChannelFactory")
        .def_static("Instance", &robot::ChannelFactory::Instance, nb::rv_policy::reference,
                    R"pbdoc(
                        Get the singleton instance of the channel factory.

                        Note: The returned instance is managed internally and should not be deleted or modified.
                    )pbdoc")
        .def("Init", nb::overload_cast<int32_t, const std::string &>(&robot::ChannelFactory::Init), nb::arg("domain_id"), nb::arg("network_interface") = "",
             R"pbdoc(
                domain_id: domain id of DDS
                network_interface: network interface of DDS, default empty string
            )pbdoc");

    nb::enum_<robot::RobotMode>(m, "RobotMode")
        .value("kUnknown", robot::RobotMode::kUnknown)
        .value("kDamping", robot::RobotMode::kDamping)
        .value("kPrepare", robot::RobotMode::kPrepare)
        .value("kWalking", robot::RobotMode::kWalking)
        .value("kCustom", robot::RobotMode::kCustom)
        .export_values();

    nb::enum_<robot::b1::JointIndex>(m, "B1JointIndex")
        .value("kHeadYaw", robot::b1::JointIndex::kHeadYaw)
        .value("kHeadPitch", robot::b1::JointIndex::kHeadPitch)
        .value("kLeftShoulderPitch", robot::b1::JointIndex::kLeftShoulderPitch)
        .value("kLeftShoulderRoll", robot::b1::JointIndex::kLeftShoulderRoll)
        .value("kLeftElbowPitch", robot::b1::JointIndex::kLeftElbowPitch)
        .value("kLeftElbowYaw", robot::b1::JointIndex::kLeftElbowYaw)
        .value("kRightShoulderPitch", robot::b1::JointIndex::kRightShoulderPitch)
        .value("kRightShoulderRoll", robot::b1::JointIndex::kRightShoulderRoll)
        .value("kRightElbowPitch", robot::b1::JointIndex::kRightElbowPitch)
        .value("kRightElbowYaw", robot::b1::JointIndex::kRightElbowYaw)
        .value("kWaist", robot::b1::JointIndex::kWaist)
        .value("kLeftHipPitch", robot::b1::JointIndex::kLeftHipPitch)
        .value("kLeftHipRoll", robot::b1::JointIndex::kLeftHipRoll)
        .value("kLeftHipYaw", robot::b1::JointIndex::kLeftHipYaw)
        .value("kLeftKneePitch", robot::b1::JointIndex::kLeftKneePitch)
        .value("kCrankUpLeft", robot::b1::JointIndex::kCrankUpLeft)
        .value("kCrankDownLeft", robot::b1::JointIndex::kCrankDownLeft)
        .value("kRightHipPitch", robot::b1::JointIndex::kRightHipPitch)
        .value("kRightHipRoll", robot::b1::JointIndex::kRightHipRoll)
        .value("kRightHipYaw", robot::b1::JointIndex::kRightHipYaw)
        .value("kRightKneePitch", robot::b1::JointIndex::kRightKneePitch)
        .value("kCrankUpRight", robot::b1::JointIndex::kCrankUpRight)
        .value("kCrankDownRight", robot::b1::JointIndex::kCrankDownRight)
        .export_values();

    m.attr("B1JointCnt") = robot::b1::kJointCnt;

    nb::enum_<robot::b1::LocoApiId>(m, "B1LocoApiId")
        .value("kChangeMode", robot::b1::LocoApiId::kChangeMode)
        .value("kMove", robot::b1::LocoApiId::kMove)
        .value("kRotateHead", robot::b1::LocoApiId::kRotateHead)
        .export_values();

    nb::enum_<robot::b1::HandAction>(m, "B1HandAction")
        .value("kHandOpen", robot::b1::HandAction::kHandOpen)
        .value("kHandClose", robot::b1::HandAction::kHandClose)
        .export_values();

    nb::enum_<robot::b1::HandIndex>(m, "B1HandIndex")
        .value("kLeftHand", robot::b1::HandIndex::kLeftHand)
        .value("kRightHand", robot::b1::HandIndex::kRightHand)
        .export_values();

    nb::enum_<robot::b1::BoosterHandType>(m, "B1HandType")
        .value("kInspireHand", robot::b1::BoosterHandType::kInspireHand)
        .value("kInspireTouchHand", robot::b1::BoosterHandType::kInspireTouchHand)
        .value("kRevoHand", robot::b1::BoosterHandType::kRevoHand)
        .value("kUnknown", robot::b1::BoosterHandType::kUnknown)
        .export_values();

    nb::enum_<robot::b1::GripperControlMode>(m, "GripperControlMode")
        .value("kPosition", robot::b1::GripperControlMode::kPosition,
               "Position mode: stops at target position or specified reaction force")
        .value("kForce", robot::b1::GripperControlMode::kForce,
               "Force mode: continues to move with specified force if target position is not reached")
        .export_values();

    nb::enum_<robot::Frame>(m, "Frame")
        .value("kUnknown", robot::Frame::kUnknown)
        .value("kBody", robot::Frame::kBody)
        .value("kHead", robot::Frame::kHead)
        .value("kLeftHand", robot::Frame::kLeftHand)
        .value("kRightHand", robot::Frame::kRightHand)
        .value("kLeftFoot", robot::Frame::kLeftFoot)
        .value("kRightFoot", robot::Frame::kRightFoot)
        .export_values();

    // Bind Position class
    nb::class_<Position>(m, "Position")
        .def(nb::init<>())
        .def(nb::init<float, float, float>(), nb::arg("x"), nb::arg("y"), nb::arg("z"))
        .def_rw("x", &Position::x_)
        .def_rw("y", &Position::y_)
        .def_rw("z", &Position::z_);

    // Bind Orientation class
    nb::class_<Orientation>(m, "Orientation")
        .def(nb::init<>())
        .def(nb::init<float, float, float>(), nb::arg("roll"), nb::arg("pitch"), nb::arg("yaw"))
        .def_rw("roll", &Orientation::roll_)
        .def_rw("pitch", &Orientation::pitch_)
        .def_rw("yaw", &Orientation::yaw_);

    // Bind Posture class
    nb::class_<Posture>(m, "Posture")
        .def(nb::init<>())
        .def(nb::init<const Position &, const Orientation &>(), nb::arg("position"), nb::arg("orientation"))
        .def_rw("position", &Posture::position_)
        .def_rw("orientation", &Posture::orientation_);

    // Bind Quaternion class
    nb::class_<Quaternion>(m, "Quaternion")
        .def(nb::init<>())
        .def(nb::init<float, float, float, float>(), nb::arg("x"), nb::arg("y"), nb::arg("z"), nb::arg("w"))
        .def_rw("x", &Quaternion::x_)
        .def_rw("y", &Quaternion::y_)
        .def_rw("z", &Quaternion::z_)
        .def_rw("w", &Quaternion::w_);

    // Bind Transform class
    nb::class_<Transform>(m, "Transform")
        .def(nb::init<>())
        .def(nb::init<const Position &, const Quaternion &>(), nb::arg("position"), nb::arg("orientation"))
        .def_rw("position", &Transform::position_)
        .def_rw("orientation", &Transform::orientation_);

    nb::class_<robot::b1::GripperMotionParameter>(m, "GripperMotionParameter")
        .def(nb::init<>())                          // Default constructor
        .def(nb::init<int32_t, int32_t, int32_t>(), // constructor with parameters
             nb::arg("position"), nb::arg("force"), nb::arg("speed"))
        .def_rw("position", &robot::b1::GripperMotionParameter::position_)
        .def_rw("force", &robot::b1::GripperMotionParameter::force_)
        .def_rw("speed", &robot::b1::GripperMotionParameter::speed_);

    nb::class_<robot::b1::GetModeResponse>(m, "GetModeResponse")
        .def(nb::init<>())
        .def_rw("mode", &robot::b1::GetModeResponse::mode_);

    nb::class_<robot::b1::DexterousFingerParameter>(m, "DexterousFingerParameter")
        .def(nb::init<>())
        .def(nb::init<int32_t, int32_t, int32_t, int32_t>(),
             nb::arg("seq"), nb::arg("angle"), nb::arg("force"), nb::arg("speed"))
        .def_rw("seq", &robot::b1::DexterousFingerParameter::seq_)
        .def_rw("angle", &robot::b1::DexterousFingerParameter::angle_)
        .def_rw("force", &robot::b1::DexterousFingerParameter::force_)
        .def_rw("speed", &robot::b1::DexterousFingerParameter::speed_);

    nb::class_<robot::b1::B1LocoClient>(m, "B1LocoClient", R"pbdoc(
        B1LocoClient is a client interface for controlling the B1 robot's locomotion and other high-level functionalities.
        It provides methods to send API requests, change robot modes, move the robot, control its head and hands, and more.
        .def("Init", nb::overload_cast<const std::string &>(&robot::b1::B1LocoClient::Init), nb::arg("robot_name"), R"pbdoc(
                /**
                 * @brief Initialize the B1LocoClient with a specific robot name.
                 * 
                 * @param robot_name The name of the robot to initialize the client for.
                 */
            )pbdoc")
        .def(nb::init<>())
        .def(
            "Init", [](robot::b1::B1LocoClient &client) {
                nb::gil_scoped_release release;
                return client.Init();
            },
            "Init")
        .def(
            "Init", [](robot::b1::B1LocoClient &client, const std::string &robot_name) {
                nb::gil_scoped_release release;
                return client.Init(robot_name);
            },
            "Init with robot name")
        .def("SendApiRequest", &robot::b1::B1LocoClient::SendApiRequest, nb::arg("api_id"), nb::arg("param"),
             R"pbdoc(
                /**
                 * @brief Send API request to B1 robot
                 * 
                 * @param api_id API_ID, you can find the API_ID in b1_api_const.hpp
                 * @param param API parameter
                 * 
                 * @return 0 if success, otherwise return error code
                 */
            )pbdoc")
        .def("ChangeMode", &robot::b1::B1LocoClient::ChangeMode, nb::arg("mode"),
             R"pbdoc(
                /**
                 * @brief Change robot mode
                 * 
                 * @param mode robot mode, options are: kDamping, kPrepare, kWalking
                 * 
                 * @return 0 if success, otherwise return error code
                 */
            )pbdoc")
        .def("GetMode", &robot::b1::B1LocoClient::GetMode, nb::arg("get_mode_response"),
             R"pbdoc(
                /**
                 * @brief Get current robot mode
                 *
                 * @param[out] get_mode_response Reference to store the response data, including:
                 *              - current_mode (RobotMode enum value)
                 *
                 * @return 0 if success, otherwise return error code
                 * @see ChangeMode() for mode switching API
                 * @see RobotMode enum for available mode definitions
                 */
               )pbdoc")
        .def("Move", &robot::b1::B1LocoClient::Move, nb::arg("vx"), nb::arg("vy"), nb::arg("vyaw"),
             R"pbdoc(
                /**
                 * @brief Move robot
                 * 
                 * @param vx linear velocity in x direction, unit: m/s
                 * @param vy linear velocity in y direction, unit: m/s
                 * @param vyaw angular velocity, unit: rad/s
                 * 
                 * @return 0 if success, otherwise return error code
                 */
            )pbdoc")
        .def("RotateHead", &robot::b1::B1LocoClient::RotateHead, nb::arg("pitch"), nb::arg("yaw"),
             R"pbdoc(
                 /**
                 * @brief Robot rotates its head
                 *
                 * @param pitch pitch angle, unit: rad
                 * @param yaw yaw angle, unit: rad
                 *
                 * @return 0 if success, otherwise return error code
                 */
            )pbdoc")
        .def("RotateHeadWithDirection", &robot::b1::B1LocoClient::RotateHeadWithDirection, nb::arg("pitch_direction"), nb::arg("yaw_direction"),
             R"pbdoc(
                 /**
                 * @brief Robot rotates its head with direction
                 *
                 * @param pitch_direction pitch direction, unit: rad
                 * @param yaw_direction yaw direction, unit: rad
                 *
                 * @return 0 if success, otherwise return error code
                 */
            )pbdoc")
        .def("WaveHand", &robot::b1::B1LocoClient::WaveHand, nb::arg("action"),
             R"pbdoc(
                 /**
                 * @brief Robot waves hand
                 *
                 * @param action hand action, options are: kHandOpen, kHandClose
                 *
                 * @return 0 if success, otherwise return error code
                 */
            )pbdoc")
        .def("Handshake", &robot::b1::B1LocoClient::Handshake, nb::arg("action"),
             R"pbdoc(
                 /**
                 * @brief Handshake
                 *
                 * @param action whether to start handshake action, options are: kHandOpen, kHandClose
                 *
                 * @return 0 if success, otherwise return error code
                 */
            )pbdoc")
        .def("MoveHandEndEffectorWithAux", &robot::b1::B1LocoClient::MoveHandEndEffectorWithAux, nb::arg("target_posture"), nb::arg("aux_posture"), nb::arg("time_millis"), nb::arg("hand_index"),
             R"pbdoc(
                /**
                 *  @brief Move hand end-effector to a target posture(position & orientation) with an auxiliary point
                 *
                 *  @param target_posture Represents the target posture in base frame (torso frame) that the hand end-effector should reach. 
                 *  It contains position & orientation.
                 *  @param aux_posture Represents the auxiliary point on the end-effector's motion arc trajectory
                 *  @param time_mills Specifies the duration, in milliseconds, for completing the movement.
                 *  @param hand_index Identifies which hand the parameter refers to (for instance, left hand or right hand).
                 *
                 *  @return 0 if success, otherwise return error code
                 */
                )pbdoc")
        .def("MoveHandEndEffector", &robot::b1::B1LocoClient::MoveHandEndEffector, nb::arg("target_posture"), nb::arg("time_millis"), nb::arg("hand_index"),
             R"pbdoc(
                /**
                 *  @brief Move hand end-effector with a target posture(position & orientation)
                 *  @deprecated **This API is deprecated and will be removed in future versions.**
                 *              Please use the new API `MoveHandEndEffectorV2` instead.
                 *  @param target_posture Represents the target posture in base frame (torso frame) that the hand end-effector should reach. 
                 *                        It contains position & orientation. 
                 *  @param time_mills Specifies the duration, in milliseconds, for completing the movement.
                 *  @param hand_index Identifies which hand the parameter refers to (for instance, left hand or right hand).
                 *
                 *  @return 0 if success, otherwise return error code
                 * 
                 *  @details
                 *  **Reason for deprecation**: This API is deprecated due to an implicit rotational offset (rot) being applied to the target orientation. 
                 *  The final orientation is calculated as orientation = rot * offset, which contradicts the parameter description of `target_posture`.
                 */
                )pbdoc")
        .def("MoveHandEndEffectorV2", &robot::b1::B1LocoClient::MoveHandEndEffectorV2, nb::arg("target_posture"), nb::arg("time_millis"), nb::arg("hand_index"),
             R"pbdoc(
                /**
                 *  @brief Move hand end-effector with a target posture(position & orientation)
                 *
                 *  @param target_posture Represents the target posture in base frame (torso frame) that the hand end-effector should reach. It contains position & orientation. 
                 *  @param time_mills Specifies the duration, in milliseconds, for completing the movement.
                 *  @param hand_index Identifies which hand the parameter refers to (for instance, left hand or right hand).
                 *
                 *  @return 0 if success, otherwise return error code
                 */
                )pbdoc")
        .def("ControlGripper", &robot::b1::B1LocoClient::ControlGripper, nb::arg("motion_param"), nb::arg("mode"), nb::arg("hand_index"),
             R"pbdoc(
                /**
                 * @brief Control gripper
                 *
                 * @param motion_param motion parameter, include position, force, velocity, see `GripperMotionParameter`
                 * @param mode gripper control mode, options are: kPosition, kForce, see `GripperControlMode`
                 * @param hand_index hand index, options are: kLeftHand, kRightHand
                 *
                 * @return 0 if success, otherwise return error code
                 */
                )pbdoc")
        .def("GetFrameTransform", &robot::b1::B1LocoClient::GetFrameTransform, nb::arg("src"), nb::arg("dst"), nb::arg("transform"),
             R"pbdoc(
                /**
                 * @brief Get frame transform
                 *
                 * @param src source frame
                 * @param dst destination frame
                 * @param transform [out] calculated transform
                 *
                 * @return 0 if success, otherwise return error code
                 */
                )pbdoc")
        .def("SwitchHandEndEffectorControlMode", &robot::b1::B1LocoClient::SwitchHandEndEffectorControlMode, nb::arg("switch_on"),
             R"pbdoc(
                /**
                 * @brief Switch hand end-effector control mode
                 * 
                 * @param switch_on true to switch on, false to switch off
                 * 
                 * @return 0 if success, otherwise return error code
                 */
                )pbdoc")
        .def("ControlDexterousHand", &robot::b1::B1LocoClient::ControlDexterousHand, nb::arg("finger_params"), nb::arg("hand_index"), nb::arg("hand_type"),
             R"pbdoc(
                /**
                 * @brief Control dexterous hand
                 *
                 * @param finger_params finger parameters, include position, force, speed, see `DexterousFingerParameter`
                 * @param hand_index hand index, options are: kLeftHand, kRightHand
                 *
                 * @return 0 if success, otherwise return error code
                 */
                )pbdoc")
        .def("GetUp", &robot::b1::B1LocoClient::GetUp,
             R"pbdoc(
                /**
                 * @brief Stand up
                 *
                 * @return 0 if success, otherwise return error code
                 */
                )pbdoc")
        .def("LieDown", &robot::b1::B1LocoClient::LieDown,
             R"pbdoc(
                /**
                 * @brief Lie down
                 *
                 * @return 0 if success, otherwise return error code
                 */
                )pbdoc");

    nb::class_<ImuState>(m, "ImuState")
        .def(nb::init<>())
        .def(nb::init<const ImuState &>())
        .def_prop_rw("rpy",
                      (const std::array<float, 3> &(ImuState::*)() const) & ImuState::rpy,
                      (void(ImuState::*)(const std::array<float, 3> &)) & ImuState::rpy)
        .def_prop_rw("gyro",
                      (const std::array<float, 3> &(ImuState::*)() const) & ImuState::gyro,
                      (void(ImuState::*)(const std::array<float, 3> &)) & ImuState::gyro)
        .def_prop_rw("acc",
                      (const std::array<float, 3> &(ImuState::*)() const) & ImuState::acc,
                      (void(ImuState::*)(const std::array<float, 3> &)) & ImuState::acc)
        .def("__eq__", &ImuState::operator==)
        .def("__ne__", &ImuState::operator!=);

    nb::class_<MotorState>(m, "MotorState")
        .def(nb::init<>())
        .def(nb::init<const MotorState &>())
        .def_prop_rw("mode",
                      (uint8_t(MotorState::*)() const) & MotorState::mode,
                      (uint8_t & (MotorState::*)()) & MotorState::mode)
        .def_prop_rw("q",
                      (float(MotorState::*)() const) & MotorState::q,
                      (float &(MotorState::*)()) & MotorState::q)
        .def_prop_rw("dq",
                      (float(MotorState::*)() const) & MotorState::dq,
                      (float &(MotorState::*)()) & MotorState::dq)
        .def_prop_rw("ddq",
                      (float(MotorState::*)() const) & MotorState::ddq,
                      (float &(MotorState::*)()) & MotorState::ddq)
        .def_prop_rw("tau_est",
                      (float(MotorState::*)() const) & MotorState::tau_est,
                      (float &(MotorState::*)()) & MotorState::tau_est)
        .def_prop_rw("temperature",
                      (uint8_t(MotorState::*)() const) & MotorState::temperature,
                      (uint8_t & (MotorState::*)()) & MotorState::temperature)
        .def_prop_rw("lost",
                      (uint32_t(MotorState::*)() const) & MotorState::lost,
                      (uint32_t & (MotorState::*)()) & MotorState::lost)
        .def_prop_rw("reserve",
                      (const std::array<uint32_t, 2> &(MotorState::*)() const) & MotorState::reserve,
                      (std::array<uint32_t, 2> & (MotorState::*)()) & MotorState::reserve)
        .def("__eq__", &MotorState::operator==)
        .def("__ne__", &MotorState::operator!=);

    nb::class_<LowState>(m, "LowState")
        .def(nb::init<>())
        .def(nb::init<const LowState &>())
        .def_prop_rw("imu_state",
                      (const ImuState &(LowState::*)() const) & LowState::imu_state,
                      (void(LowState::*)(const ImuState &)) & LowState::imu_state)
        .def_prop_rw("motor_state_parallel",
                      (const std::vector<MotorState> &(LowState::*)() const) & LowState::motor_state_parallel,
                      (void(LowState::*)(const std::vector<MotorState> &)) & LowState::motor_state_parallel)
        .def_prop_rw("motor_state_serial",
                      (const std::vector<MotorState> &(LowState::*)() const) & LowState::motor_state_serial,
                      (void(LowState::*)(const std::vector<MotorState> &)) & LowState::motor_state_serial)
        .def("__eq__", &LowState::operator==)
        .def("__ne__", &LowState::operator!=);

    nb::class_<MotorCmd>(m, "MotorCmd")
        .def(nb::init<>())
        .def(nb::init<const MotorCmd &>())
        .def_prop_rw("mode",
                      (uint8_t(MotorCmd::*)() const) & MotorCmd::mode,
                      (void(MotorCmd::*)(uint8_t)) & MotorCmd::mode)
        .def_prop_rw("q",
                      (float(MotorCmd::*)() const) & MotorCmd::q,
                      (void(MotorCmd::*)(float)) & MotorCmd::q)
        .def_prop_rw("dq",
                      (float(MotorCmd::*)() const) & MotorCmd::dq,
                      (void(MotorCmd::*)(float)) & MotorCmd::dq)
        .def_prop_rw("tau",
                      (float(MotorCmd::*)() const) & MotorCmd::tau,
                      (void(MotorCmd::*)(float)) & MotorCmd::tau)
        .def_prop_rw("kp",
                      (float(MotorCmd::*)() const) & MotorCmd::kp,
                      (void(MotorCmd::*)(float)) & MotorCmd::kp)
        .def_prop_rw("kd",
                      (float(MotorCmd::*)() const) & MotorCmd::kd,
                      (void(MotorCmd::*)(float)) & MotorCmd::kd)
        .def_prop_rw("weight",
                      (float(MotorCmd::*)() const) & MotorCmd::weight,
                      (void(MotorCmd::*)(float)) & MotorCmd::weight)
        .def("__eq__", &MotorCmd::operator==)
        .def("__ne__", &MotorCmd::operator!=);

    nb::enum_<CmdType>(m, "LowCmdType")
        .value("PARALLEL", CmdType::PARALLEL)
        .value("SERIAL", CmdType::SERIAL)
        .export_values();

    nb::class_<LowCmd>(m, "LowCmd")
        .def(nb::init<>())
        .def(nb::init<const LowCmd &>())
        .def_prop_rw("cmd_type",
                      (CmdType(LowCmd::*)() const) & LowCmd::cmd_type,
                      (void(LowCmd::*)(CmdType)) & LowCmd::cmd_type)
        .def_prop_rw("motor_cmd",
                      (const std::vector<MotorCmd> &(LowCmd::*)() const) & LowCmd::motor_cmd,
                      (void(LowCmd::*)(const std::vector<MotorCmd> &)) & LowCmd::motor_cmd)
        .def("__eq__", &LowCmd::operator==)
        .def("__ne__", &LowCmd::operator!=);

    nb::class_<robot::b1::B1LowStateSubscriber>(m, "B1LowStateSubscriber")
        .def(nb::init<const std::function<void(const LowState*)> &>(), nb::arg("handler"), R"pbdoc(
                 /**
                 * @brief init low state subscriber with callback handler
                 *
                 * @param handler callback handler of low state, the handler should accept one parameter of type LowState
                 *
                 */
            )pbdoc")
        .def("InitChannel", &robot::b1::B1LowStateSubscriber::InitChannel, "Init low state subscription channel")
        .def("CloseChannel", &robot::b1::B1LowStateSubscriber::CloseChannel, "Close low state subscription channel")
        .def("GetChannelName", &robot::b1::B1LowStateSubscriber::GetChannelName, "Get low state subscription channel name");

    nb::class_<robot::b1::B1LowHandTouchDataScriber>(m, "B1LowHandTouchDataScriber")
        .def(nb::init<const std::function<void(const HandTouchData*)> &>(), nb::arg("handler"), R"pbdoc(
                 /**
                 * @brief init hand touch data subscriber with callback handler
                 *
                 * @param handler callback handler of hand touch data, the handler should accept one parameter of type LowState
                 *
                 */
            )pbdoc")
        .def("InitChannel", &robot::b1::B1LowHandTouchDataScriber::InitChannel, "Init low state subscription channel")
        .def("CloseChannel", &robot::b1::B1LowHandTouchDataScriber::CloseChannel, "Close low state subscription channel")
        .def("GetChannelName", &robot::b1::B1LowHandTouchDataScriber::GetChannelName, "Get low state subscription channel name");

    nb::class_<robot::b1::B1LowHandDataScriber>(m, "B1LowHandDataScriber")
        .def(nb::init<const std::function<void(const HandReplyData*)> &>(), nb::arg("handler"), R"pbdoc(
                 /**
                 * @brief init hand data subscriber with callback handler
                 *
                 * @param handler callback handler of hand data, the handler should accept one parameter of type LowState
                 *
                 */
            )pbdoc")
        .def("InitChannel", &robot::b1::B1LowHandDataScriber::InitChannel, "Init low state subscription channel")
        .def("CloseChannel", &robot::b1::B1LowHandDataScriber::CloseChannel, "Close low state subscription channel")
        .def("GetChannelName", &robot::b1::B1LowHandDataScriber::GetChannelName, "Get low state subscription channel name");

    nb::class_<robot::b1::B1LowCmdPublisher>(m, "B1LowCmdPublisher")
        .def(nb::init<>())
        .def("InitChannel", &robot::b1::B1LowCmdPublisher::InitChannel, "Init low cmd publication channel")
        .def("Write", &robot::b1::B1LowCmdPublisher::Write, nb::arg("msg"), R"pbdoc(
                 /**
                 * @brief write low cmd message into channel, i.e. publish low cmd message
                 *
                 * @param msg LowCmd
                 *
                 */
            )pbdoc")
        .def("CloseChannel", &robot::b1::B1LowCmdPublisher::CloseChannel, "Close low cmd publication channel")
        .def("GetChannelName", &robot::b1::B1LowCmdPublisher::GetChannelName, "Get low cmd publication channel name");

    nb::class_<Odometer>(m, "Odometer")
        .def(nb::init<>())
        .def_prop_rw("x",
                      (float(Odometer::*)() const) & Odometer::x,
                      (void(Odometer::*)(float)) & Odometer::x)
        .def_prop_rw("y",
                      (float(Odometer::*)() const) & Odometer::y,
                      (void(Odometer::*)(float)) & Odometer::y)
        .def_prop_rw("theta",
                      (float(Odometer::*)() const) & Odometer::theta,
                      (void(Odometer::*)(float)) & Odometer::theta);

    nb::class_<robot::b1::B1OdometerStateSubscriber>(m, "B1OdometerStateSubscriber")
        .def(nb::init<const std::function<void(const Odometer*)> &>(), nb::arg("handler"), R"pbdoc(
                 /**
                 * @brief init odometer state subscriber with callback handler
                 *
                 * @param handler callback handler of odom state, the handler should accept one parameter of type Odometer
                 *
                 */
            )pbdoc")
        .def("InitChannel", &robot::b1::B1OdometerStateSubscriber::InitChannel, "Init odometer subscription channel")
        .def("CloseChannel", &robot::b1::B1OdometerStateSubscriber::CloseChannel, "Close odometer subscription channel")
        .def("GetChannelName", &robot::b1::B1OdometerStateSubscriber::GetChannelName, "Get odometer subscription channel name");

    nb::class_<HandReplyParam>(m, "HandReplyParam")
        .def(nb::init<>())
        .def(nb::init<const HandReplyParam &>())
        .def_prop_rw("angle",
                      (int32_t(HandReplyParam::*)() const) & HandReplyParam::angle,
                      (int32_t & (HandReplyParam::*)()) & HandReplyParam::angle)
        .def_prop_rw("force",
                      (int32_t(HandReplyParam::*)() const) & HandReplyParam::force,
                      (int32_t & (HandReplyParam::*)()) & HandReplyParam::force)
        .def_prop_rw("current",
                      (int32_t(HandReplyParam::*)() const) & HandReplyParam::current,
                      (int32_t & (HandReplyParam::*)()) & HandReplyParam::current)
        .def_prop_rw("error",
                      (int32_t(HandReplyParam::*)() const) & HandReplyParam::error,
                      (int32_t & (HandReplyParam::*)()) & HandReplyParam::error)
        .def_prop_rw("status",
                      (int32_t(HandReplyParam::*)() const) & HandReplyParam::status,
                      (int32_t & (HandReplyParam::*)()) & HandReplyParam::status)
        .def_prop_rw("temp",
                      (int32_t(HandReplyParam::*)() const) & HandReplyParam::temp,
                      (int32_t & (HandReplyParam::*)()) & HandReplyParam::temp)
        .def_prop_rw("seq",
                      (int32_t(HandReplyParam::*)() const) & HandReplyParam::seq,
                      (int32_t & (HandReplyParam::*)()) & HandReplyParam::seq)
        .def("__eq__", &HandReplyParam::operator==)
        .def("__ne__", &HandReplyParam::operator!=);

    nb::class_<HandReplyData>(m, "HandReplyData")
        .def(nb::init<>())
        .def(nb::init<const HandReplyData &>())
        .def_prop_rw("hand_index",
                      (int32_t(HandReplyData::*)() const) & HandReplyData::hand_index,
                      (int32_t & (HandReplyData::*)()) & HandReplyData::hand_index)
        .def_prop_rw("hand_type",
                      (int32_t(HandReplyData::*)() const) & HandReplyData::hand_type,
                      (int32_t & (HandReplyData::*)()) & HandReplyData::hand_type)
        .def_prop_rw("hand_data",
                      (const std::vector<HandReplyParam> &(HandReplyData::*)() const) & HandReplyData::hand_data,
                      (void(HandReplyData::*)(const std::vector<HandReplyParam> &)) & HandReplyData::hand_data)
        .def("__eq__", &HandReplyData::operator==)
        .def("__ne__", &HandReplyData::operator!=);

    nb::class_<HandTouchParam>(m, "HandTouchParam")
        .def(nb::init<>())
        .def(nb::init<const HandTouchParam &>())
        .def_prop_rw("finger_one",
                      (const std::vector<uint8_t> &(HandTouchParam::*)() const) & HandTouchParam::finger_one,
                      (void(HandTouchParam::*)(const std::vector<uint8_t> &)) & HandTouchParam::finger_one)
        .def_prop_rw("finger_two",
                      (const std::vector<uint8_t> &(HandTouchParam::*)() const) & HandTouchParam::finger_two,
                      (void(HandTouchParam::*)(const std::vector<uint8_t> &)) & HandTouchParam::finger_two)
        .def_prop_rw("finger_three",
                      (const std::vector<uint8_t> &(HandTouchParam::*)() const) & HandTouchParam::finger_three,
                      (void(HandTouchParam::*)(const std::vector<uint8_t> &)) & HandTouchParam::finger_three)
        .def_prop_rw("finger_four",
                      (const std::vector<uint8_t> &(HandTouchParam::*)() const) & HandTouchParam::finger_four,
                      (void(HandTouchParam::*)(const std::vector<uint8_t> &)) & HandTouchParam::finger_four)
        .def_prop_rw("finger_five",
                      (const std::vector<uint8_t> &(HandTouchParam::*)() const) & HandTouchParam::finger_five,
                      (void(HandTouchParam::*)(const std::vector<uint8_t> &)) & HandTouchParam::finger_five)
        .def_prop_rw("finger_palm",
                      (const std::vector<uint8_t> &(HandTouchParam::*)() const) & HandTouchParam::finger_palm,
                      (void(HandTouchParam::*)(const std::vector<uint8_t> &)) & HandTouchParam::finger_palm)

        .def("__eq__", &HandTouchParam::operator==)
        .def("__ne__", &HandTouchParam::operator!=);

    nb::class_<HandTouchData>(m, "HandTouchData")
        .def(nb::init<>())
        .def(nb::init<const HandTouchData &>())
        .def_prop_rw("hand_index",
                      (int32_t(HandTouchData::*)() const) & HandTouchData::hand_index,
                      (int32_t & (HandTouchData::*)()) & HandTouchData::hand_index)
        .def_prop_rw("hand_type",
                      (int32_t(HandTouchData::*)() const) & HandTouchData::hand_type,
                      (int32_t & (HandTouchData::*)()) & HandTouchData::hand_type)

        .def_prop_rw("touch_data",
                      (const HandTouchParam &(HandTouchData::*)() const) & HandTouchData::touch_data,
                      (void(HandTouchData::*)(const HandTouchParam &)) & HandTouchData::touch_data)
        .def("__eq__", &HandTouchData::operator==)
        .def("__ne__", &HandTouchData::operator!=);
#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}
