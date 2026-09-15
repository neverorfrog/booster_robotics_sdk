#ifndef __BOOSTER_ROBOTICS_SDK_RPC_OPERATION_CLIENT_HPP__
#define __BOOSTER_ROBOTICS_SDK_RPC_OPERATION_CLIENT_HPP__

#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <string>

#include <booster/idl/rpc/RpcRespMsg.h>
#include <booster/robot/channel/channel_subscriber.hpp>
#include <booster/robot/rpc/rpc_client.hpp>
#include <booster/robot/rpc/rpc_operation_types.hpp>

namespace booster {
namespace robot {

struct SendApiRequestOptions {
    using StartCallback = std::function<void(const ApiOperationStartResult &)>;
    using ProgressCallback = std::function<void(const ApiOperationHandle::SharedPtr &, const ApiOperationProgress &)>;
    using ResultCallback = std::function<void(const ApiOperationResult &)>;

    StartCallback start_callback;
    ProgressCallback progress_callback;
    ResultCallback result_callback;
    int64_t start_timeout_ms{1000};
};

struct RpcOperationClientConfig {
    std::string rpc_channel_name;
    std::string event_channel_name;
};

class RpcOperationClient {
public:
    RpcOperationClient() = default;
    ~RpcOperationClient();

    void Init(const RpcOperationClientConfig &config);
    void Stop();

    bool WaitForService(
        int64_t timeout_ms = RpcClient::kDefaultWaitForServiceTimeoutMs,
        bool require_event_path = true);

    ApiOperationSubmitResult AsyncSendApiRequest(
        int64_t api_id,
        const std::string &param);

    ApiOperationSubmitResult AsyncSendApiRequest(
        int64_t api_id,
        const std::string &param,
        const SendApiRequestOptions &options);

    std::shared_future<int32_t> AsyncCancelOperation(
        const ApiOperationHandle::SharedPtr &operation_handle);

private:
    struct OperationContext;
    struct SharedState;

    bool WaitForEventEndpoint(int64_t timeout_ms) const;
    void DdsEventMsgHandler(const void *msg);
    static void FinishOperation(
        const std::shared_ptr<SharedState> &state,
        const std::string &operation_id,
        const ApiOperationResult &result);
    static void RemoveOperation(
        const std::shared_ptr<SharedState> &state,
        const std::string &operation_id);

    RpcOperationClientConfig config_;
    std::shared_ptr<RpcClient> rpc_client_;
    std::shared_ptr<ChannelSubscriber<booster_msgs::msg::RpcRespMsg>> event_subscriber_;
    std::shared_ptr<SharedState> state_;
};

}
} // namespace booster::robot

#endif // __BOOSTER_ROBOTICS_SDK_RPC_OPERATION_CLIENT_HPP__
