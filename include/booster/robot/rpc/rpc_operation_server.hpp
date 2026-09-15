#ifndef __BOOSTER_ROBOTICS_SDK_RPC_OPERATION_SERVER_HPP__
#define __BOOSTER_ROBOTICS_SDK_RPC_OPERATION_SERVER_HPP__

#include <cstdint>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <booster/idl/rpc/RpcRespMsg.h>
#include <booster/robot/channel/channel_publisher.hpp>
#include <booster/robot/rpc/rpc_operation_types.hpp>
#include <booster/robot/rpc/rpc_server.hpp>

namespace booster {
namespace robot {

struct RpcOperationServerConfig {
    std::string rpc_channel_name;
    std::string event_channel_name;
    bool event_reliable{true};
    int64_t progress_interval_ms{500};
};

struct OperationRequestContext {
    std::string req_id;
    std::string operation_id;
    int64_t api_id{0};
    std::string param;
    std::chrono::steady_clock::time_point now;
};

struct OperationContext {
    std::string operation_id;
    int64_t api_id{0};
    std::shared_ptr<void> payload;
    bool cancel_requested{false};
    std::chrono::steady_clock::time_point created_at;
    std::chrono::steady_clock::time_point deadline;
    std::chrono::steady_clock::time_point last_progress_time;
    std::string last_progress_body;
};

struct OperationStartDecision {
    int32_t status{kRpcStatusCodeInvalid};
    std::string message;
    std::shared_ptr<void> payload;
    std::chrono::steady_clock::time_point deadline;
};

struct OperationEvalResult {
    bool finished{false};
    ApiOperationResultCode code{ApiOperationResultCode::kUnknown};
    int64_t status{kRpcStatusCodeInvalid};
    std::string message;
    std::string body;
    std::vector<std::string> progress_bodies;
};

class RpcOperationServer : public RpcServer {
public:
    RpcOperationServer() = default;
    ~RpcOperationServer() override;

    void Init(const RpcOperationServerConfig &config);
    void Stop();
    void UpdateOperations();

protected:
    booster::robot::Response HandleOperationRequest(const booster::robot::Request &req);

    virtual OperationStartDecision Start(const OperationRequestContext &ctx);
    virtual int32_t Cancel(
        const OperationContext &ctx,
        const std::shared_ptr<void> &payload);
    virtual OperationEvalResult Tick(
        const OperationContext &ctx,
        const std::shared_ptr<void> &payload);
    virtual std::string BuildProgress(
        const OperationContext &ctx,
        const std::shared_ptr<void> &payload);

    int32_t PublishOperationProgress(const ApiOperationProgress &progress);
    int32_t PublishOperationResult(const ApiOperationResult &result);
    int32_t PublishOperationEvent(
        const std::string &operation_id,
        int64_t api_id,
        ApiOperationEventType event_type,
        ApiOperationResultCode result_code,
        int64_t status,
        const std::string &message,
        const std::string &body);

private:
    booster::robot::Response HandleStartOperation(const booster::robot::Request &req);
    booster::robot::Response HandleCancelOperation(const booster::robot::Request &req);
    void PublishOperationResult(
        const OperationContext &operation,
        const OperationEvalResult &eval_result);

    int64_t progress_interval_ms_{500};
    std::mutex operation_mutex_;
    std::unordered_map<std::string, OperationContext> pending_operations_;
    std::shared_ptr<ChannelPublisher<booster_msgs::msg::RpcRespMsg>> operation_event_publisher_;
};

}
} // namespace booster::robot

#endif // __BOOSTER_ROBOTICS_SDK_RPC_OPERATION_SERVER_HPP__
