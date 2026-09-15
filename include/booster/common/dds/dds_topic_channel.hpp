#ifndef __BOOSTER_DDS_TOPIC_CHANNEL_HPP__
#define __BOOSTER_DDS_TOPIC_CHANNEL_HPP__

#include <iostream>

#include <booster/common/dds/dds_entity.hpp>

namespace booster {
namespace common {

template <typename MSG>
class DdsTopicChannel {
public:
    DdsTopicChannel() = default;
    ~DdsTopicChannel() {
        CloseReader();
        CloseWriter();
    }

    void SetTopic(DdsTopicPtr topic) {
        topic_ = topic;
    }

    void SetWriter(
        const DdsPublisherPtr &publisher,
        const DdsDataWriterQos &qos) {
        if (publisher == nullptr || topic_ == nullptr) {
            std::cerr << "Failed to create writer: publisher or topic is not initialized." << std::endl;
            return;
        }
        auto raw_writer = publisher->create_datawriter(topic_.get(), qos);
        if (raw_writer == nullptr) {
            std::cerr << "Failed to create writer." << std::endl;
            return;
        }
        CloseWriter();
        writer_owner_ = publisher;
        writer_ = DdsWriterPtr(raw_writer, [](DdsWriter *writer) {});
    }

    void SetReader(
        const DdsSubscriberPtr &subscriber,
        const DdsDataReaderQos &qos,
        const DdsReaderCallback &cb,
        const DdsReaderExecutorOptions &executor_options = {}) {
        if (subscriber == nullptr || topic_ == nullptr) {
            std::cerr << "Failed to create reader: subscriber or topic is not initialized." << std::endl;
            return;
        }
        auto listener = std::make_shared<DdsReaderListener<MSG>>();
        listener->SetExecutorOptions(executor_options);
        listener->SetCallback(cb);
        auto raw_reader = subscriber->create_datareader(topic_.get(), qos, listener.get());
        if (raw_reader == nullptr) {
            std::cerr << "Failed to create reader." << std::endl;
            return;
        }
        CloseReader();
        listener_ = listener;
        reader_owner_ = subscriber;
        reader_ = DdsReaderPtr(raw_reader, [](DdsReader *reader) {});
    }

    DdsWriterPtr GetWriter() const {
        return writer_;
    }

    DdsReaderPtr GetReader() const {
        return reader_;
    }

    void CloseWriter() {
        if (writer_owner_ != nullptr && writer_ != nullptr) {
            writer_owner_->delete_datawriter(writer_.get());
        }
        writer_.reset();
        writer_owner_.reset();
    }

    void CloseReader() {
        if (reader_owner_ != nullptr && reader_ != nullptr) {
            reader_owner_->delete_datareader(reader_.get());
        }
        reader_.reset();
        reader_owner_.reset();
        listener_.reset();
    }

    DdsReaderExecutorMetrics GetReaderExecutorMetrics() const {
        if (listener_ == nullptr) {
            return DdsReaderExecutorMetrics();
        }
        return listener_->GetExecutorMetrics();
    }

    size_t GetMatchedSubscriptionsCount() const {
        if (writer_ == nullptr) {
            return 0;
        }

        DdsPublicationMatchedStatus status;
        if (writer_->get_publication_matched_status(status) != ReturnCode_t::RETCODE_OK) {
            return 0;
        }

        return status.current_count > 0 ? static_cast<size_t>(status.current_count) : 0;
    }

    size_t GetMatchedPublicationsCount() const {
        if (reader_ == nullptr) {
            return 0;
        }

        DdsSubscriptionMatchedStatus status;
        if (reader_->get_subscription_matched_status(status) != ReturnCode_t::RETCODE_OK) {
            return 0;
        }

        return status.current_count > 0 ? static_cast<size_t>(status.current_count) : 0;
    }

    bool Write(MSG *msg) {
        if (writer_ == nullptr) {
            std::cerr << "Write failed: writer is not initialized." << std::endl;
            return false;
        }
        if (msg == nullptr) {
            std::cerr << "Write failed: message is null." << std::endl;
            return false;
        }
        return writer_->write(msg);
    }

private:
    DdsWriterPtr writer_;
    DdsReaderPtr reader_;
    DdsTopicPtr topic_;
    DdsPublisherPtr writer_owner_;
    DdsSubscriberPtr reader_owner_;
    DdsReaderListenerPtr<MSG> listener_;
};

template <typename MSG>
using DdsTopicChannelPtr = std::shared_ptr<DdsTopicChannel<MSG>>;

}
} // namespace booster::common

#endif // __BOOSTER_DDS_TOPIC_CHANNEL_HPP__
