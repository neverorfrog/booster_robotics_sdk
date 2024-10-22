#ifndef __BOOSTER_ROBOTICS_SDK_B1_LOCO_API_HPP__
#define __BOOSTER_ROBOTICS_SDK_B1_LOCO_API_HPP__

#include <string>
#include <booster/third_party/nlohmann_json/json.hpp>
#include <booster/robot/b1/b1_api_const.hpp>
#include <booster/robot/common/robot_mode.hpp>

namespace booster {
namespace robot {
namespace b1 {

/* service name */
const std::string LOCO_SERVICE_NAME = "loco";

/*API version*/
const std::string LOCO_API_VERSION = "1.0.0.1";

/*API ID */
enum class LocoApiId {
    kChangeMode = 2000,
    kMove = 2001,
    kRotateHead = 2004,
    kWaveHand = 2005,
    kRotateHeadWithDirection = 2006,
    kLieDown = 2007,
    kGetUp = 2008,
};

class RotateHeadParameter {
public:
    RotateHeadParameter() = default;
    RotateHeadParameter(float pitch, float yaw) :
        pitch_(pitch),
        yaw_(yaw) {
    }

public:
    void FromJson(nlohmann::json &json) {
        pitch_ = json["pitch"];
        yaw_ = json["yaw"];
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["pitch"] = pitch_;
        json["yaw"] = yaw_;
        return json;
    }

public:
    float pitch_;
    float yaw_;
};

class ChangeModeParameter {
public:
    ChangeModeParameter() = default;
    ChangeModeParameter(booster::robot::RobotMode mode) :
        mode_(mode) {
    }

public:
    void FromJson(nlohmann::json &json) {
        mode_ = static_cast<booster::robot::RobotMode>(json["mode"]);
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["mode"] = static_cast<int>(mode_);
        return json;
    }

public:
    booster::robot::RobotMode mode_;
};

class MoveParameter {
public:
    MoveParameter() = default;
    MoveParameter(float vx, float vy, float vyaw) :
        vx_(vx),
        vy_(vy),
        vyaw_(vyaw) {
    }

public:
    void FromJson(nlohmann::json &json) {
        vx_ = json["vx"];
        vy_ = json["vy"];
        vyaw_ = json["vyaw"];
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["vx"] = vx_;
        json["vy"] = vy_;
        json["vyaw"] = vyaw_;
        return json;
    }

public:
    float vx_;
    float vy_;
    float vyaw_;
};

class RotateHeadWithDirectionParameter {
public:
    RotateHeadWithDirectionParameter() = default;
    RotateHeadWithDirectionParameter(int pitch_direction, int yaw_direction) :
        pitch_direction_(pitch_direction),
        yaw_direction_(yaw_direction) {
    }

public:
    void FromJson(nlohmann::json &json) {
        pitch_direction_ = json["pitch_direction"];
        yaw_direction_ = json["yaw_direction"];
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["pitch_direction"] = pitch_direction_;
        json["yaw_direction"] = yaw_direction_;
        return json;
    }

public:
    int pitch_direction_;
    int yaw_direction_;
};

class WaveHandParameter {
public:
    WaveHandParameter() = default;
    WaveHandParameter(booster::robot::b1::HandIndex hand_index, booster::robot::b1::HandAction hand_action) :
        hand_action_(hand_action), hand_index_(hand_index) {
    }

public:
    void FromJson(nlohmann::json &json) {
        hand_index_ = static_cast<booster::robot::b1::HandIndex>(json["hand_index"]);
        hand_action_ = static_cast<booster::robot::b1::HandAction>(json["hand_action"]);
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["hand_index"] = static_cast<int>(hand_index_);
        json["hand_action"] = static_cast<int>(hand_action_);
        return json;
    }

public:
    booster::robot::b1::HandIndex hand_index_;
    booster::robot::b1::HandAction hand_action_;
};
}
}
} // namespace booster::robot::b1

#endif // __BOOSTER_ROBOTICS_SDK_B1_LOCO_API_HPP__