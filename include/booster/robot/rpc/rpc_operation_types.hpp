#ifndef __BOOSTER_ROBOTICS_SDK_RPC_OPERATION_TYPES_HPP__
#define __BOOSTER_ROBOTICS_SDK_RPC_OPERATION_TYPES_HPP__

#include <cstdint>
#include <memory>
#include <string>

#include <booster/robot/rpc/error.hpp>

namespace booster {
namespace robot {

enum class ApiOperationEventType : int32_t {
    kUnknown = 0,
    kProgress = 1,
    kFinished = 2,
};

enum class ApiOperationResultCode : int32_t {
    kUnknown = 0,
    kSucceeded = 1,
    kAborted = 2,
    kCanceled = 3,
    kRejected = 4,
    kTimeout = 5,
};

class ApiOperationHandle {
public:
    using SharedPtr = std::shared_ptr<ApiOperationHandle>;

    ApiOperationHandle(std::string operation_id, int64_t api_id);

    const std::string &GetOperationId() const;
    int64_t GetApiId() const;

private:
    std::string operation_id_;
    int64_t api_id_{0};
};

struct ApiOperationProgress {
    std::string operation_id;
    int64_t api_id{0};
    int64_t status{0};
    std::string message;
    std::string body;
};

struct ApiOperationResult {
    ApiOperationResultCode code{ApiOperationResultCode::kUnknown};
    std::string operation_id;
    int64_t api_id{0};
    int64_t status{0};
    std::string message;
    std::string body;
};

struct ApiOperationStartResult {
    // Server start status from the start RPC. Success means the server accepted the operation.
    int32_t status{kRpcStatusCodeInvalid};
    std::string operation_id;
    int64_t api_id{0};
    std::string message;
    ApiOperationHandle::SharedPtr handle;
};

struct ApiOperationSubmitResult {
    // Local submit status. Success means the SDK created operation_id/handle and started the start RPC.
    int32_t status{kRpcStatusCodeInvalid};
    std::string operation_id;
    std::string message;
    ApiOperationHandle::SharedPtr handle;
};

}
} // namespace booster::robot

#endif // __BOOSTER_ROBOTICS_SDK_RPC_OPERATION_TYPES_HPP__
