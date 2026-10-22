#ifndef __BOOSTER_ROBOTICS_SDK_REQUEST_HEADER_HPP__
#define __BOOSTER_ROBOTICS_SDK_REQUEST_HEADER_HPP__

#include <cstdint>
#include <string>
#include <booster/third_party/nlohmann_json/json.hpp>

namespace booster {
namespace robot {

enum class RpcCallMode : int32_t {
    kRequestResponse = 0,
    kOperation = 1,
    kFireAndForget = 2,
};

enum class RpcOperationCommand : int32_t {
    kNone = 0,
    kStart = 1,
    kCancel = 2,
};

/** @brief Header identifying an RPC method and its delivery or operation mode. */
class RequestHeader {
public:
    /** @brief Constructs a request-response header with default API id 0. */
    RequestHeader() = default;
    /** @brief Constructs a header for @p api_id. */
    RequestHeader(int64_t api_id) :
        api_id_(api_id) {
    }

    /** @brief Sets the service API identifier. */
    void SetApiId(int64_t api_id) {
        api_id_ = api_id;
    }

    /** @brief Returns the service API identifier. */
    int64_t GetApiId() const {
        return api_id_;
    }

    /** @brief Sets the RPC delivery and response mode. */
    void SetCallMode(RpcCallMode call_mode) {
        call_mode_ = call_mode;
    }

    /** @brief Returns the RPC delivery and response mode. */
    RpcCallMode GetCallMode() const {
        return call_mode_;
    }

    /** @brief Sets the lifecycle command for an asynchronous operation. */
    void SetOperationCommand(RpcOperationCommand operation_command) {
        operation_command_ = operation_command;
    }

    /** @brief Returns the asynchronous operation lifecycle command. */
    RpcOperationCommand GetOperationCommand() const {
        return operation_command_;
    }

    /** @brief Sets the request identifier used for correlation. */
    void SetReqId(const std::string &req_id) {
        req_id_ = req_id;
    }

    /** @brief Returns the request identifier used for correlation. */
    const std::string &GetReqId() const {
        return req_id_;
    }

    /** @brief Sets the identifier of the asynchronous operation. */
    void SetOperationId(const std::string &operation_id) {
        operation_id_ = operation_id;
    }

    /** @brief Returns the identifier of the asynchronous operation. */
    const std::string &GetOperationId() const {
        return operation_id_;
    }

public:
    /** @brief Loads the API id and optional operation metadata from JSON. */
    void FromJson(nlohmann::json &json) {
        api_id_ = json["api_id"];
        call_mode_ = static_cast<RpcCallMode>(
            json.value("call_mode", static_cast<int32_t>(RpcCallMode::kRequestResponse)));
        operation_command_ = static_cast<RpcOperationCommand>(
            json.value("operation_command", static_cast<int32_t>(RpcOperationCommand::kNone)));
        req_id_ = json.value("req_id", std::string());
        operation_id_ = json.value("operation_id", std::string());
    }

    /** @brief Serializes this header to JSON. */
    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["api_id"] = api_id_;
        json["call_mode"] = static_cast<int32_t>(call_mode_);
        if (operation_command_ != RpcOperationCommand::kNone) {
            json["operation_command"] = static_cast<int32_t>(operation_command_);
        }
        if (!req_id_.empty()) {
            json["req_id"] = req_id_;
        }
        if (!operation_id_.empty()) {
            json["operation_id"] = operation_id_;
        }
        return json;
    }

private:
    int64_t api_id_ = 0;
    RpcCallMode call_mode_{RpcCallMode::kRequestResponse};
    RpcOperationCommand operation_command_{RpcOperationCommand::kNone};
    std::string req_id_;
    std::string operation_id_;
};

}
} // namespace booster::robot

#endif // __BOOSTER_ROBOTICS_SDK_REQUEST_HEADER_HPP__
