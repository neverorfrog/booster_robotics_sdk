#pragma once

#include <stdexcept>
#include <string>
#include <utility>

#include <booster/third_party/nlohmann_json/json.hpp>

namespace booster {
namespace robot {

/**
 * @file api.hpp
 * @brief Data types and RPC identifiers for AI chat and local voice services.
 *
 * This header defines request payloads for AiClient and LuiClient. Payload
 * objects can be serialized with ToJson() and populated with FromJson(). The AI
 * chat service provides conversational ASR, LLM, and TTS orchestration, while
 * the LUI service exposes standalone ASR and TTS control.
 *
 * @note The JSON conversion functions use nlohmann::json. Unless a function
 * explicitly wraps the error in std::runtime_error, malformed or incomplete
 * JSON may raise a nlohmann::json::exception.
 *
 * @par Model support
 * @note Supported model: K1 | T1 | T2
 * @note AI chat, direct speech, standalone ASR, and standalone TTS require
 * deployed AI/LUI, audio, and network services.
 * @note Face tracking supported model: K1 | T1
 * @note Face tracking requires an NVIDIA perception board. In the current RTC
 * service, standalone face-tracking RPC handlers acknowledge requests without
 * activating or stopping tracking.
 * @warning The face-tracking behavior described here reflects the RTC service
 * implementation shipped with this SDK revision. Re-verify the behavior after
 * upgrading the SDK, RTC service, or robot software.
 *
 * @see AiClient
 * @see LuiClient
 */

/**
 * @brief Numeric identifiers for AI chat service operations.
 *
 * Applications normally call the corresponding convenience method on
 * AiClient. These identifiers are exposed for generic RPC access and protocol
 * diagnostics.
 */
enum class AiApiId {
    kStartAiChat = 2000,      ///< Start an AI chat session; supported model: K1 | T1 | T2.
    kStopAiChat = 2001,       ///< Stop the active AI chat session; supported model: K1 | T1 | T2.
    kSpeak = 2002,            ///< Speak text during an active AI chat session; supported model: K1 | T1 | T2.
    kStartFaceTracking = 2003, ///< Acknowledged but currently has no tracking effect.
    kStopFaceTracking = 2004,  ///< Acknowledged but currently has no tracking effect.
};

/**
 * @brief Text-to-speech configuration for an AI chat session.
 *
 * The AI backend uses the voice identifier to choose a speaker and uses the
 * bracket-filter list to remove non-spoken annotations from generated text.
 *
 * @note Supported model: K1 | T1 | T2
 */
class TtsConfig {
public:
    /** @brief Constructs a configuration with an empty voice and filter list. */
    TtsConfig() = default;

    /**
     * @brief Constructs an AI chat TTS configuration.
     * @param voice_type Backend-defined TTS voice identifier.
     * @param ignore_bracket_text Bracket categories to remove before speech
     * synthesis. See ignore_bracket_text_ for supported values.
     */
    TtsConfig(const std::string &voice_type, const std::vector<int8_t> &ignore_bracket_text) :
        voice_type_(voice_type), ignore_bracket_text_(ignore_bracket_text) {
    }

public:
    /**
     * @brief Populates this configuration from JSON.
     * @param json JSON object containing string field `voice_type` and integer
     * array field `ignore_bracket_text`.
     * @throws std::runtime_error If a required field is missing or has an
     * incompatible JSON type.
     */
    void FromJson(nlohmann::json &json) {
        try {
            voice_type_ = json.at("voice_type").get<std::string>();

            auto int_array = json.at("ignore_bracket_text").get<std::vector<int>>();
            ignore_bracket_text_.assign(int_array.begin(), int_array.end());
        } catch (const nlohmann::json::exception &e) {
            throw std::runtime_error("TtsConfig JSON error: " + std::string(e.what()));
        }
    }

    /**
     * @brief Serializes this configuration to JSON.
     * @return JSON object containing `voice_type` and `ignore_bracket_text`.
     */
    nlohmann::json ToJson() const {
        return {
            {"voice_type", voice_type_},
            {"ignore_bracket_text", std::vector<int>(ignore_bracket_text_.begin(), ignore_bracket_text_.end())}};
    }

public:
    /**
     * Backend-defined voice identifier controlling language, speaker, and
     * timbre. Examples include `zh_male_wennuanahu_moon_bigtts` for Chinese and
     * American English, and `zh_female_shuangkuaisisi_emo_v2_mars_bigtts` for
     * Chinese and British English. Available identifiers are service-dependent.
     */
    std::string voice_type_;

    /**
     * Bracket categories whose enclosed text is removed before synthesis.
     * Supported values are 1 for full-width parentheses, 2 for ASCII
     * parentheses, 3 for full-width square brackets, 4 for ASCII square
     * brackets, and 5 for ASCII curly braces. The system prompt should place
     * non-spoken annotations inside the selected bracket categories.
     */
    std::vector<int8_t> ignore_bracket_text_;
};

/**
 * @brief Large-language-model persona and greeting configuration.
 * @note Supported model: K1 | T1 | T2
 */
class LlmConfig {
public:
    /** @brief Constructs a configuration with empty strings. */
    LlmConfig() = default;

    /**
     * @brief Constructs an LLM configuration.
     * @param system_prompt System instruction defining the assistant persona and
     * behavior.
     * @param welcome_msg Greeting supplied when the chat session starts.
     * @param prompt_name Backend-defined named persona or prompt preset. An
     * empty string selects no named preset.
     */
    LlmConfig(const std::string &system_prompt, const std::string &welcome_msg, const std::string &prompt_name = "") :
        system_prompt_(system_prompt), welcome_msg_(welcome_msg), prompt_name_(prompt_name) {
    }

    /**
     * @brief Populates this configuration from JSON.
     * @param json JSON object containing string fields `system_prompt`,
     * `welcome_msg`, and `prompt_name`.
     * @throws nlohmann::json::exception If a required field is missing or has
     * an incompatible type.
     */
    void FromJson(nlohmann::json &json) {
        system_prompt_ = json.at("system_prompt").get<std::string>();
        welcome_msg_ = json.at("welcome_msg").get<std::string>();
        prompt_name_ = json.at("prompt_name").get<std::string>();
    }

    /**
     * @brief Serializes this configuration to JSON.
     * @return JSON object containing the system prompt, greeting, and preset
     * name.
     */
    nlohmann::json ToJson() const {
        return {
            {"system_prompt", system_prompt_},
            {"welcome_msg", welcome_msg_},
            {"prompt_name", prompt_name_},
        };
    }

public:
    std::string system_prompt_; ///< System instruction defining persona and behavior.
    std::string welcome_msg_;   ///< Greeting supplied when the session starts.
    std::string prompt_name_;   ///< Backend-defined named prompt or persona preset.
};

/**
 * @brief Speech-recognition interruption settings for an AI chat session.
 *
 * These settings apply when interruption mode is enabled and determine when
 * recognized user speech may interrupt robot speech.
 *
 * @note Supported model: K1 | T1 | T2
 */
class AsrConfig {
public:
    /**
     * @brief Constructs an empty configuration.
     * @warning interrupt_speech_duration_ is not initialized. Assign it before
     * serializing or sending the configuration.
     */
    AsrConfig() = default;

    /**
     * @brief Constructs ASR interruption settings.
     * @param interrupt_speech_duration Continuous speech duration threshold in
     * milliseconds for automatic interruption.
     * @param interrupt_keywords Keywords that immediately trigger interruption
     * when recognized.
     */
    AsrConfig(int32_t interrupt_speech_duration, std::vector<std::string> interrupt_keywords) :
        interrupt_speech_duration_(interrupt_speech_duration), interrupt_keywords_(interrupt_keywords) {
    }

    /**
     * @brief Populates this configuration from JSON.
     * @param json JSON object containing integer field
     * `interrupt_speech_duration` and string array field `interrupt_keywords`.
     * @throws nlohmann::json::exception If a required field is missing or has
     * an incompatible type.
     */
    void FromJson(nlohmann::json &json) {
        interrupt_speech_duration_ = json["interrupt_speech_duration"];
        interrupt_keywords_ = json["interrupt_keywords"];
    }

    /**
     * @brief Serializes this configuration to JSON.
     * @return JSON object containing the interruption duration and keywords.
     */
    nlohmann::json ToJson() const {
        return {{"interrupt_speech_duration", interrupt_speech_duration_},
                {"interrupt_keywords", interrupt_keywords_}};
    }

public:
    int32_t interrupt_speech_duration_; ///< Automatic interruption threshold in milliseconds.
    std::vector<std::string> interrupt_keywords_; ///< Interruption-triggering keywords.
};

/**
 * @brief Request payload for starting an AI chat session.
 *
 * The payload combines an optional AgentHub persona identifier, interruption
 * behavior, conversational TTS, LLM settings, ASR settings, and optional
 * face-tracking intent.
 *
 * @note Supported model: K1 | T1 | T2
 * @note Face tracking supported model: K1 | T1
 * @note Face tracking requires an NVIDIA perception board. The current RTC
 * service does not activate tracking from this option.
 * @warning The face-tracking behavior is version-sensitive; re-verify it after
 * upgrading the SDK, RTC service, or robot software.
 */
class StartAiChatParameter {
public:
    /**
     * @brief Constructs an otherwise empty request with interruption disabled.
     * @warning asr_config_.interrupt_speech_duration_ and
     * enable_face_tracking_ are not initialized. Assign all request fields
     * before calling ToJson() or AiClient::StartAiChat().
     */
    StartAiChatParameter() = default;

public:
    /**
     * @brief Populates this request from JSON.
     * @param json JSON object containing optional `persona_id` and required
     * `interrupt_mode`, `enable_face_tracking`, `tts_config`, `llm_config`,
     * and `asr_config` fields.
     * @throws std::runtime_error If the nested TTS configuration is invalid.
     * @throws nlohmann::json::exception If another required field is missing or
     * has an incompatible type.
     */
    void FromJson(nlohmann::json &json) {
        persona_id_ = json.value("persona_id", "");
        interrupt_mode_ = json["interrupt_mode"];
        enable_face_tracking_ = json["enable_face_tracking"];
        tts_config_.FromJson(json["tts_config"]);
        llm_config_.FromJson(json["llm_config"]);
        asr_config_.FromJson(json["asr_config"]);
    }

    /**
     * @brief Serializes this request to the AI chat RPC representation.
     * @return JSON object containing all AI chat configuration sections.
     */
    nlohmann::json ToJson() const {
        nlohmann::json json;
        if (!persona_id_.empty()) {
            json["persona_id"] = persona_id_;
        }
        json["interrupt_mode"] = interrupt_mode_;
        json["asr_config"] = asr_config_.ToJson();
        json["llm_config"] = llm_config_.ToJson();
        json["tts_config"] = tts_config_.ToJson();
        json["enable_face_tracking"] = enable_face_tracking_;
        return json;
    }

public:
    std::string persona_id_;      ///< Optional AgentHub persona identifier.
    bool interrupt_mode_ = false; ///< Whether recognized user speech can interrupt robot speech.
    TtsConfig tts_config_;        ///< Conversational speech-synthesis settings.
    LlmConfig llm_config_;        ///< Persona, greeting, and named-prompt settings.
    AsrConfig asr_config_;        ///< Speech interruption thresholds and keywords.
    bool enable_face_tracking_;   ///< Face-tracking intent; see the model-support note above.
};

/**
 * @brief Request payload for speaking text through an active AI chat session.
 * @note Supported model: K1 | T1 | T2
 */
class SpeakParameter {
public:
    /** @brief Constructs a request with an empty message. */
    SpeakParameter() = default;

    /** @brief Constructs a speech request. @param msg Text to synthesize and play. */
    SpeakParameter(const std::string &msg) :
        msg_(msg) {
    }

public:
    /**
     * @brief Populates this request from JSON.
     * @param json JSON object containing string field `msg`.
     * @throws nlohmann::json::exception If `msg` is missing or is not a string.
     */
    void FromJson(nlohmann::json &json) {
        msg_ = json["msg"];
    }

    /** @brief Serializes the message. @return JSON object containing `msg`. */
    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["msg"] = msg_;
        return json;
    }

public:
    std::string msg_; ///< Non-empty text to synthesize during an active AI chat session.
};

// Input limits shared by the LUI speech APIs. The LUI service re-validates the
// same limits, so keep these values in sync with
// booster_lui/src/util/lui_input_limits.h.
//
// Text length is counted in Unicode code points so that Chinese and English
// text get the same budget. Audio size is counted in decoded bytes, not in
// base64 characters, so that the limit means the same thing for both the
// `file` and `pcm` input types.
constexpr int32_t kLuiMaxSynthesizeTextCodePoints = 1000;
constexpr int32_t kLuiMaxRecognizeAudioDurationMs = 120 * 1000;
constexpr int64_t kLuiMaxRecognizeAudioBytes = 6 * 1024 * 1024;

/**
 * @brief Count Unicode code points in a UTF-8 string.
 *
 * Continuation bytes (0b10xxxxxx) are skipped, so each code point is counted
 * once regardless of how many bytes encode it. Invalid UTF-8 is counted
 * leniently rather than rejected; the service performs the authoritative check.
 */
inline int64_t CountUtf8CodePoints(const std::string &text) {
    int64_t count = 0;
    for (const char raw_byte : text) {
        if ((static_cast<unsigned char>(raw_byte) & 0xC0) != 0x80) {
            ++count;
        }
    }
    return count;
}

/**
 * @brief Decoded byte count of a base64 payload, derived from its length.
 *
 * Avoids decoding just to measure size. Padding is subtracted so the result is
 * exact for well-formed base64.
 */
inline int64_t EstimateBase64DecodedBytes(const std::string &base64_data) {
    int64_t padding = 0;
    for (auto it = base64_data.rbegin(); it != base64_data.rend() && *it == '='; ++it) {
        ++padding;
    }
    const int64_t length = static_cast<int64_t>(base64_data.size());
    return length / 4 * 3 - padding;
}

/**
 * @brief Numeric identifiers for standalone LUI speech operations.
 *
 * LUI provides speech recognition and synthesis independently of an AI chat
 * session. Applications normally call the corresponding LuiClient methods.
 *
 * The LUI service keeps a matching copy of these values in
 * booster_lui/src/local_api.h. Values are duplicated rather than shared, so any
 * change here must be applied there as well.
 */
enum class LuiApiId {
    kStartAsr = 1000,    ///< Start microphone ASR; supported model: K1 | T1 | T2.
    kStopAsr = 1001,     ///< Stop standalone ASR; supported model: K1 | T1 | T2.
    kStartTts = 1050,    ///< Start a standalone TTS session; supported model: K1 | T1 | T2.
    kStopTts = 1051,     ///< Stop the standalone TTS session; supported model: K1 | T1 | T2.
    kSendTtsText = 1052, ///< Queue text for the active TTS session; supported model: K1 | T1 | T2.
    kSynthesizeSpeech = 1100,
    kRecognizeAudioOnce = 1101,
    kStartAudioRecognizer = 1102,
    kStopAudioRecognizer = 1103,
    kRecognizeAudioInSession = 1104,
};

class LuiAsrStartRequest {
public:
    LuiAsrStartRequest() = default;
    LuiAsrStartRequest(std::string session_id, std::string client_id) :
        session_id_(std::move(session_id)),
        client_id_(std::move(client_id)) {
    }

    void FromJson(const nlohmann::json &json) {
        try {
            session_id_ = json.at("session_id").get<std::string>();
            client_id_ = json.at("client_id").get<std::string>();
        } catch (const nlohmann::json::exception &e) {
            throw std::runtime_error(
                "LuiAsrStartRequest JSON error: " + std::string(e.what()));
        }
    }

    nlohmann::json ToJson() const {
        return {
            {"session_id", session_id_},
            {"client_id", client_id_},
        };
    }

    std::string session_id_;
    std::string client_id_;
};

class LuiAsrStopRequest {
public:
    LuiAsrStopRequest() = default;
    LuiAsrStopRequest(std::string session_id, std::string client_id) :
        session_id_(std::move(session_id)),
        client_id_(std::move(client_id)) {
    }

    void FromJson(const nlohmann::json &json) {
        try {
            session_id_ = json.at("session_id").get<std::string>();
            client_id_ = json.at("client_id").get<std::string>();
        } catch (const nlohmann::json::exception &e) {
            throw std::runtime_error(
                "LuiAsrStopRequest JSON error: " + std::string(e.what()));
        }
    }

    nlohmann::json ToJson() const {
        return {
            {"session_id", session_id_},
            {"client_id", client_id_},
        };
    }

    std::string session_id_;
    std::string client_id_;
};

/**
 * @brief Voice configuration for a standalone LUI TTS session.
 * @note Supported model: K1 | T1 | T2
 */
class LuiTtsConfig {
public:
    /** @brief Constructs a configuration with an empty voice identifier. */
    LuiTtsConfig() = default;

    /**
     * @brief Constructs a standalone TTS configuration.
     * @param voice_type Backend-defined TTS voice identifier.
     */
    LuiTtsConfig(const std::string &voice_type) :
        voice_type_(voice_type) {
    }

public:
    /**
     * @brief Populates this configuration from JSON.
     * @param json JSON object containing string field `voice_type`.
     * @throws std::runtime_error If the field is missing or is not a string.
     */
    void FromJson(nlohmann::json &json) {
        try {
            voice_type_ = json.at("voice_type").get<std::string>();
        } catch (const nlohmann::json::exception &e) {
            throw std::runtime_error("LuiTtsConfig JSON error: " + std::string(e.what()));
        }
    }

    /** @brief Serializes the voice selection. @return JSON object containing `voice_type`. */
    nlohmann::json ToJson() const {
        return {
            {"voice_type", voice_type_}};
    }

public:
    /**
     * Backend-defined voice identifier controlling language, speaker, and
     * timbre. Availability is determined by the deployed TTS service.
     */
    std::string voice_type_;
};

class LuiTtsStopRequest {
public:
    LuiTtsStopRequest() = default;
    LuiTtsStopRequest(std::string session_id, std::string client_id) :
        session_id_(std::move(session_id)),
        client_id_(std::move(client_id)) {
    }

    void FromJson(const nlohmann::json &json) {
        try {
            session_id_ = json.at("session_id").get<std::string>();
            client_id_ = json.at("client_id").get<std::string>();
        } catch (const nlohmann::json::exception &e) {
            throw std::runtime_error(
                "LuiTtsStopRequest JSON error: " + std::string(e.what()));
        }
    }

    nlohmann::json ToJson() const {
        return {
            {"session_id", session_id_},
            {"client_id", client_id_},
        };
    }

    std::string session_id_;
    std::string client_id_;
};

/**
 * @brief Text payload for a standalone LUI TTS session.
 * @note Supported model: K1 | T1 | T2
 */
class LuiTtsParameter {
public:
    /** @brief Constructs a payload with empty text. */
    LuiTtsParameter() = default;

    /** @brief Constructs a TTS text payload. @param text Non-empty text to synthesize. */
    LuiTtsParameter(const std::string &text) :
        text_(text) {
    }

public:
    /**
     * @brief Populates this payload from JSON.
     * @param json JSON object containing string field `text`.
     * @throws std::runtime_error If the field is missing or is not a string.
     */
    void FromJson(nlohmann::json &json) {
        try {
            text_ = json.at("text").get<std::string>();
        } catch (const nlohmann::json::exception &e) {
            throw std::runtime_error("LuiTtsParameter JSON error: " + std::string(e.what()));
        }
    }

    /** @brief Serializes the TTS text. @return JSON object containing `text`. */
    nlohmann::json ToJson() const {
        return {
            {"text", text_}};
    }

public:
    std::string text_; ///< Non-empty text to synthesize in the active LUI TTS session.
};

class LuiSynthesizeSpeechRequest {
public:
    LuiSynthesizeSpeechRequest() = default;
    LuiSynthesizeSpeechRequest(const std::string &text,
                               const std::string &voice_type = "default",
                               double speed = 1.0,
                               bool playback = false) :
        text_(text), voice_type_(voice_type), speed_(speed), playback_(playback) {
    }

    void FromJson(const nlohmann::json &json) {
        try {
            text_ = json.at("text").get<std::string>();
            if (json.contains("voice_type")) {
                voice_type_ = json.at("voice_type").get<std::string>();
            }
            if (json.contains("speed")) {
                speed_ = json.at("speed").get<double>();
            }
            if (json.contains("playback")) {
                playback_ = json.at("playback").get<bool>();
            }
        } catch (const nlohmann::json::exception &e) {
            throw std::runtime_error("LuiSynthesizeSpeechRequest JSON error: " + std::string(e.what()));
        }
    }

    nlohmann::json ToJson() const {
        return {
            {"text", text_},
            {"voice_type", voice_type_},
            {"speed", speed_},
            {"playback", playback_},
        };
    }

public:
    std::string text_;
    std::string voice_type_ = "default";
    double speed_ = 1.0; // Speech speed ratio: 1.0 = normal, 0.5 = slowest, 2.0 = fastest.
    bool playback_ = false;
};

class LuiSynthesizeSpeechResponse {
public:
    void FromJson(const nlohmann::json &json) {
        try {
            audio_base64_ = json.at("audio_base64").get<std::string>();
            sample_rate_hz_ = json.at("sample_rate_hz").get<int32_t>();
            channels_ = json.at("channels").get<int32_t>();
            bits_per_sample_ = json.at("bits_per_sample").get<int32_t>();
            format_ = json.at("format").get<std::string>();
        } catch (const nlohmann::json::exception &e) {
            throw std::runtime_error("LuiSynthesizeSpeechResponse JSON error: " + std::string(e.what()));
        }
    }

    nlohmann::json ToJson() const {
        return {
            {"audio_base64", audio_base64_},
            {"sample_rate_hz", sample_rate_hz_},
            {"channels", channels_},
            {"bits_per_sample", bits_per_sample_},
            {"format", format_},
        };
    }

public:
    std::string audio_base64_;
    int32_t sample_rate_hz_ = 16000;
    int32_t channels_ = 1;
    int32_t bits_per_sample_ = 16;
    std::string format_ = "pcm_s16le";
};

class LuiRecognizeAudioRequest {
public:
    LuiRecognizeAudioRequest() = default;

    /**
     * @brief Build a file-based recognition request.
     *
     * The file path must point to a `.wav` or `.mp3` audio file.
     *
     * @param file_path Audio file path.
     * @return Recognition request configured for `input_type=file`.
     */
    static LuiRecognizeAudioRequest FromFile(const std::string &file_path) {
        LuiRecognizeAudioRequest req;
        req.input_type_ = "file";
        req.file_path_ = file_path;
        return req;
    }

    /**
     * @brief Build a raw PCM recognition request.
     *
     * The payload must be base64-encoded 16kHz mono 16-bit little-endian raw PCM
     * and the format should remain `pcm_s16le`.
     *
     * @param audio_base64 Base64-encoded raw PCM payload.
     * @param sample_rate_hz PCM sample rate. Must be 16000.
     * @param channels PCM channel count. Must be 1.
     * @param bits_per_sample PCM bit depth. Must be 16.
     * @param format PCM format string. Must be `pcm_s16le`.
     * @return Recognition request configured for `input_type=pcm`.
     */
    static LuiRecognizeAudioRequest FromPcmBase64(const std::string &audio_base64,
                                                  int32_t sample_rate_hz = 16000,
                                                  int32_t channels = 1,
                                                  int32_t bits_per_sample = 16,
                                                  const std::string &format = "pcm_s16le") {
        LuiRecognizeAudioRequest req;
        req.input_type_ = "pcm";
        req.audio_base64_ = audio_base64;
        req.sample_rate_hz_ = sample_rate_hz;
        req.channels_ = channels;
        req.bits_per_sample_ = bits_per_sample;
        req.format_ = format;
        return req;
    }

    void FromJson(const nlohmann::json &json) {
        try {
            input_type_ = json.at("input_type").get<std::string>();
            if (json.contains("file_path")) {
                file_path_ = json.at("file_path").get<std::string>();
            }
            if (json.contains("audio_base64")) {
                audio_base64_ = json.at("audio_base64").get<std::string>();
            }
            if (json.contains("sample_rate_hz")) {
                sample_rate_hz_ = json.at("sample_rate_hz").get<int32_t>();
            }
            if (json.contains("channels")) {
                channels_ = json.at("channels").get<int32_t>();
            }
            if (json.contains("bits_per_sample")) {
                bits_per_sample_ = json.at("bits_per_sample").get<int32_t>();
            }
            if (json.contains("format")) {
                format_ = json.at("format").get<std::string>();
            }
        } catch (const nlohmann::json::exception &e) {
            throw std::runtime_error("LuiRecognizeAudioRequest JSON error: " + std::string(e.what()));
        }
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["input_type"] = input_type_;
        if (!file_path_.empty()) {
            json["file_path"] = file_path_;
        }
        if (!audio_base64_.empty()) {
            json["audio_base64"] = audio_base64_;
        }
        json["sample_rate_hz"] = sample_rate_hz_;
        json["channels"] = channels_;
        json["bits_per_sample"] = bits_per_sample_;
        json["format"] = format_;
        return json;
    }

public:
    std::string input_type_;
    std::string file_path_;
    std::string audio_base64_;
    int32_t sample_rate_hz_ = 16000;
    int32_t channels_ = 1;
    int32_t bits_per_sample_ = 16;
    std::string format_ = "pcm_s16le";
};

class LuiRecognizeAudioResponse {
public:
    void FromJson(const nlohmann::json &json) {
        try {
            text_ = json.at("text").get<std::string>();
        } catch (const nlohmann::json::exception &e) {
            throw std::runtime_error("LuiRecognizeAudioResponse JSON error: " + std::string(e.what()));
        }
    }

    nlohmann::json ToJson() const {
        return {
            {"text", text_},
        };
    }

public:
    std::string text_;
};

}
} // namespace booster::robot
