#ifdef OBS_ENCODER

#include "examples/peerconnection/client/win_obs_video_encoder.h"
#include "examples/peerconnection/client/win_obs_adapter.h"

#include "rtc_base/timeutils.h"
#include "rtc_base/logging.h"
#include "media/base/h264_profile_level_id.h"
#include "modules/video_coding/include/video_codec_interface.h"
#include "modules/include/module_common_types.h"
#include "common_video/h264/h264_common.h"
#include "common_video/libyuv/include/webrtc_libyuv.h"

#include <sys/timeb.h>

#define SIMPLE_ENCODER_X264_LOWCPU             "x264_lowcpu"
#define SIMPLE_ENCODER_QSV                     "qsv"
#define SIMPLE_ENCODER_NVENC                   "nvenc"
#define SIMPLE_ENCODER_AMD                     "amd"

namespace webrtc {

const int32_t kMaxObsVideoEncoderFps = 30;

WinObsVideoEncoder::WinObsVideoEncoder()
    : clock_(webrtc::Clock::GetRealTimeClock()),
      delta_ntp_internal_ms_(clock_->CurrentNtpInMilliseconds() - clock_->TimeInMilliseconds()) {

}

WinObsVideoEncoder::~WinObsVideoEncoder() {

}

//WebRTC interfaces.
int32_t WinObsVideoEncoder::InitEncode(
    const webrtc::VideoCodec* codec_settings,
    int32_t number_of_cores,
    size_t max_payload_size) {
    if (encode_thread_ == NULL) {
        encode_thread_ = rtc::Thread::Create();
        encode_thread_->SetName("obs_encode_thread", nullptr);
        RTC_CHECK(encode_thread_->Start()) << "Failed to start obs encode thread";
    }
    
    int32_t ret = encode_thread_->Invoke<int32_t>(
        RTC_FROM_HERE,
        rtc::Bind(&WinObsVideoEncoder::init_encode_on_codec_thread,
            this,
            codec_settings->width,
            codec_settings->height,
            codec_settings->targetBitrate,
            codec_settings->maxFramerate));
    return ret;
}

int32_t WinObsVideoEncoder::RegisterEncodeCompleteCallback(
    webrtc::EncodedImageCallback* callback) {
    int32_t ret = encode_thread_->Invoke<int32_t>(
        RTC_FROM_HERE,
        rtc::Bind(&WinObsVideoEncoder::register_encode_complete_callback_on_codec_thread, this, callback));
    return ret;
}

int32_t WinObsVideoEncoder::Release() {
    int32_t ret = encode_thread_->Invoke<int32_t>(
        RTC_FROM_HERE,
        rtc::Bind(&WinObsVideoEncoder::release_on_codec_thread, this));
    return ret;
}

int32_t WinObsVideoEncoder::Encode(
    const webrtc::VideoFrame& frame,
    const webrtc::CodecSpecificInfo* codec_specific_info,
    const std::vector<webrtc::FrameType>* frame_types) {
    int32_t ret = encode_thread_->Invoke<int32_t>(
        RTC_FROM_HERE,
        rtc::Bind(&WinObsVideoEncoder::encode_on_codec_thread, this, frame, frame_types->front()));
    return ret;
}

int32_t WinObsVideoEncoder::SetChannelParameters(
    uint32_t packet_loss,
    int64_t rtt) {
    return 0;
}

int32_t WinObsVideoEncoder::SetRateAllocation(
    const webrtc::BitrateAllocation& rate_allocation,
    uint32_t frame_rate) {
    return 0;
}

bool WinObsVideoEncoder::open() {
    ConfigFile &config = WinObsAdapter::get_config();
    const char *encoder_name = config_get_string(config, "SimpleOutput", "StreamEncoder");
    if (encoder_name == NULL) {
        RTC_LOG(LS_ERROR) << "OBS StreamEncoder empty, check obs.ini.";
        return false;
    }

    //Search and load encoder.
    bool ret = true;
    if (strcmp(encoder_name, SIMPLE_ENCODER_QSV) == 0) {
        ret = load_streaming_preset_h264("obs_qsv11");
    } else if (strcmp(encoder_name, SIMPLE_ENCODER_AMD) == 0) {
        ret = load_streaming_preset_h264("amd_amf_h264");
    } else if (strcmp(encoder_name, SIMPLE_ENCODER_NVENC) == 0) {
        ret = load_streaming_preset_h264("ffmpeg_nvenc");
    } else {
        ret = load_streaming_preset_h264("obs_x264");
    }

    if (!ret) {
        return false;
    }

    //Configure encoder.
    configure_encoder(config);

    //Initialize encoder.
    if (!obs_encoder_initialize(h264_encoder_)) {
        return false;
    }

    opened = true;
    return true;
}

bool WinObsVideoEncoder::close() {
    if (h264_encoder_ != NULL) {
        obs_encoder_release(h264_encoder_);
    }
    opened = false;
    return true;
}

bool WinObsVideoEncoder::start() {
    bool ret = true;
    do {
        if (h264_encoder_ == NULL) {
            ret = false;
            break;
        }

        obs_encoder_t *encoder = h264_encoder_;
        pthread_mutex_lock(&encoder->init_mutex);
        ret = start_internal();
        pthread_mutex_unlock(&encoder->init_mutex);
    } while (0);
    return ret;
}

bool WinObsVideoEncoder::stop() {
    bool ret = true;
    do {
        if (h264_encoder_ == NULL) {
            ret = false;
            break;
        }

        obs_encoder_t *encoder = h264_encoder_;
        pthread_mutex_lock(&encoder->init_mutex);
        bool destroyed = stop_internal();
        if (!destroyed) {
            pthread_mutex_unlock(&encoder->init_mutex);
        }
    } while (0);
    return ret;
}

bool WinObsVideoEncoder::start_internal() {
    struct encoder_callback cb = { false, new_encoded_packet, this };
    bool first = false;

    obs_encoder_t *encoder = h264_encoder_;
    if (encoder == NULL) {
        return false;
    }

    if (!encoder->context.data) {
        return false;
    }

    pthread_mutex_lock(&encoder->callbacks_mutex);

    first = (encoder->callbacks.num == 0);

    size_t idx = get_callback_idx(encoder, new_encoded_packet, this);
    if (idx == DARRAY_INVALID) {
        da_push_back(encoder->callbacks, &cb);
    }

    pthread_mutex_unlock(&encoder->callbacks_mutex);

    if (first) {
        encoder->cur_pts = 0;
        os_atomic_set_bool(&encoder->active, true);
    }

    started = true;
    return true;
}

bool WinObsVideoEncoder::stop_internal() {
    bool   last = false;
    size_t idx;

    obs_encoder_t *encoder = h264_encoder_;
    if (encoder == NULL) {
        return false;
    }

    pthread_mutex_lock(&encoder->callbacks_mutex);

    idx = get_callback_idx(encoder, new_encoded_packet, this);
    if (idx != DARRAY_INVALID) {
        da_erase(encoder->callbacks, idx);
        last = (encoder->callbacks.num == 0);
    }

    pthread_mutex_unlock(&encoder->callbacks_mutex);
    started = false;

    if (last) {
        obs_encoder_shutdown(encoder);
        os_atomic_set_bool(&encoder->active, false);
        encoder->initialized = false;

        if (encoder->destroy_on_stop) {
            pthread_mutex_unlock(&encoder->init_mutex);
            obs_encoder_actually_destroy(encoder);
            return true;
        }
    }

    return false;
}

void WinObsVideoEncoder::configure_encoder(const ConfigFile& config) {
    obs_data_t *h264Settings = obs_data_create();

    int32_t videoBitrate    = (int32_t)config_get_uint(config, "SimpleOutput", "VBitrate");
    bool advanced           = config_get_bool(config, "SimpleOutput", "UseAdvanced");
    bool enforceBitrate     = config_get_bool(config, "SimpleOutput", "EnforceBitrate");
    const char *custom      = config_get_string(config, "SimpleOutput", "x264Settings");
    const char *encoder     = config_get_string(config, "SimpleOutput", "StreamEncoder");
    const char *rateControl = config_get_string(config, "SimpleOutput", "RateControl");
    const char *presetType  = NULL;
    const char *preset      = NULL;

    if (strcmp(encoder, SIMPLE_ENCODER_QSV) == 0) {
        presetType = "QSVPreset";
    } else if (strcmp(encoder, SIMPLE_ENCODER_AMD) == 0) {
        presetType = "AMDPreset";
        update_streaming_settings_amd(h264Settings, videoBitrate);
    } else if (strcmp(encoder, SIMPLE_ENCODER_NVENC) == 0) {
        presetType = "NVENCPreset";
    } else {
        presetType = "Preset";
        preferred_video_format_ = VIDEO_FORMAT_I420;
    }

    obs_encoder_set_preferred_video_format(h264_encoder_, preferred_video_format_);
    if (preferred_video_format_ == VIDEO_FORMAT_NV12) {
        frame_length_ = width_ * height_ + ((width_ + 1) >> 1) * ((height_ + 1) >> 1) * 2;
        nv12_buff_.reset(new uint8_t[frame_length_]);
        nv12_y_plane_ = (uint8_t *)nv12_buff_.get();
        nv12_uv_plane_ = nv12_y_plane_ + width_ * height_;
    }

    preset = config_get_string(config, "SimpleOutput", presetType);
    if (advanced) {
        obs_data_set_string(h264Settings, "preset", preset);
        obs_data_set_string(h264Settings, "x264opts", custom);
    }

    if (rateControl == NULL) {
        rateControl = "CBR";
    }

    obs_data_set_string(h264Settings, "rate_control", rateControl);
    obs_data_set_int(h264Settings, "bitrate", videoBitrate);

    if (advanced && !enforceBitrate) {
        obs_data_set_int(h264Settings, "bitrate", videoBitrate);
    }

    //Disable b-frames
    obs_data_set_int(h264Settings, "bf", 0);

    //Default 250 frames' time.
    obs_data_set_int(h264Settings, "keyint_sec", 10);

    //For QSV immediately encoding first frame in real time circumstance.
    obs_data_set_int(h264Settings, "async_depth", 1);

    video_t *video = obs_get_video();
    obs_encoder_update(h264_encoder_, h264Settings);
    obs_data_release(h264Settings);
    obs_encoder_set_video(h264_encoder_, video);
}

void WinObsVideoEncoder::new_encoded_packet(void *param, struct encoder_packet *packet) {
    WinObsVideoEncoder *encoder = (WinObsVideoEncoder*)param;
    if (encoder != NULL) {
        encoder->on_new_encoded_packet(packet);
    }
}

void WinObsVideoEncoder::on_new_encoded_packet(struct encoder_packet *packet) {
    if (!input_frame_infos_.empty()) {
        const InputFrameInfo& frame_info    = input_frame_infos_.front();
        output_timestamp_                   = frame_info.frame_timestamp;
        output_render_time_ms_              = frame_info.frame_render_time_ms;
        input_frame_infos_.pop_front();
    }

    size_t payload_size = packet->size;
    uint8_t *payload    = packet->data;

    const webrtc::VideoCodecType codec_type = webrtc::kVideoCodecH264;
    webrtc::EncodedImageCallback::Result callback_result(webrtc::EncodedImageCallback::Result::OK);
    if (callback_) {
        std::unique_ptr<webrtc::EncodedImage> image(new webrtc::EncodedImage(payload, payload_size, payload_size));
        image->_encodedWidth    = width_;
        image->_encodedHeight   = height_;
        image->_timeStamp       = output_timestamp_;
        image->capture_time_ms_ = output_render_time_ms_;
        image->rotation_        = webrtc::kVideoRotation_0;
        image->_frameType       = (packet->keyframe ? webrtc::kVideoFrameKey : webrtc::kVideoFrameDelta);
        image->_completeFrame   = true;

        webrtc::CodecSpecificInfo info;
        memset(&info, 0, sizeof(info));
        info.codecType = codec_type;

        // Generate a header describing a single fragment.
        webrtc::RTPFragmentationHeader header;
        memset(&header, 0, sizeof(header));
        h264_bitstream_parser_.ParseBitstream(payload, payload_size);
        int qp;
        if (h264_bitstream_parser_.GetLastSliceQp(&qp)) {
            image->qp_ = qp;
        }
        // For H.264 search for start codes.
        const std::vector<webrtc::H264::NaluIndex> nalu_idxs = webrtc::H264::FindNaluIndices(payload, payload_size);
        if (nalu_idxs.empty()) {
            RTC_LOG(LS_ERROR) << "Start code is not found!";
            char data[1024] = {0};
            sprintf(data, 
                "Data: %02x %02x %02x %02x %02x %02x",
                image->_buffer[0], image->_buffer[1], image->_buffer[2],
                image->_buffer[3], image->_buffer[4], image->_buffer[5]);
            RTC_LOG(LS_ERROR) << data;
            return;
        }
        header.VerifyAndAllocateFragmentationHeader(nalu_idxs.size());
        for (size_t i = 0; i < nalu_idxs.size(); i++) {
            header.fragmentationOffset[i]   = nalu_idxs[i].payload_start_offset;
            header.fragmentationLength[i]   = nalu_idxs[i].payload_size;
            header.fragmentationPlType[i]   = 0;
            header.fragmentationTimeDiff[i] = 0;
        }

        callback_result = callback_->OnEncodedImage(*image, &info, &header);
    }
}

bool WinObsVideoEncoder::load_streaming_preset_h264(const char *encoderId) {
    h264_encoder_ = obs_video_encoder_create(encoderId, "bee_h264_stream", nullptr, nullptr);
    if (!h264_encoder_) {
        RTC_LOG(LS_ERROR) << "Failed to create h264 streaming encoder (simple output)";
        return false;
    }
    obs_encoder_release(h264_encoder_);
    return true;
}

void WinObsVideoEncoder::update_streaming_settings_amd(obs_data_t *settings, int bitrate) {
    // Static Properties
    obs_data_set_int(settings, "Usage", 0);
    obs_data_set_int(settings, "Profile", 100); // High

    // Rate Control Properties
    obs_data_set_int(settings, "RateControlMethod", 3);
    obs_data_set_int(settings, "Bitrate.Target", bitrate);
    obs_data_set_int(settings, "FillerData", 1);
    obs_data_set_int(settings, "VBVBuffer", 1);
    obs_data_set_int(settings, "VBVBuffer.Size", bitrate);

    // Picture Control Properties
    obs_data_set_double(settings, "KeyframeInterval", 2.0);
    obs_data_set_int(settings, "BFrame.Pattern", 0);
}

size_t WinObsVideoEncoder::get_callback_idx(
    const struct obs_encoder *encoder,
    void(*new_packet)(void *param, struct encoder_packet *packet),
    void *param) {
    for (size_t i = 0; i < encoder->callbacks.num; i++) {
        struct encoder_callback *cb = encoder->callbacks.array + i;
        if (cb->new_packet == new_packet && cb->param == param) {
            return i;
        }
    }

    return DARRAY_INVALID;
}

void WinObsVideoEncoder::obs_encoder_actually_destroy(obs_encoder_t *encoder) {
    if (encoder) {
        RTC_LOG(LS_ERROR) << "Encoder " << encoder->context.name << " destroyed";

        if (encoder->context.data) {
            encoder->info.destroy(encoder->context.data);
        }

        da_free(encoder->callbacks);
        pthread_mutex_destroy(&encoder->init_mutex);
        pthread_mutex_destroy(&encoder->callbacks_mutex);
        obs_context_data_free(&encoder->context);

        if (encoder->owns_info_id) {
            bfree((void*)encoder->info.id);
        }
        bfree(encoder);
    }
}

int32_t WinObsVideoEncoder::init_encode_on_codec_thread(
    int32_t width,
    int32_t height,
    int32_t target_bitrate,
    int32_t fps) {
    int32_t ret = 0;
    do {
        width_                          = width;
        height_                         = height;
        last_frame_received_ms_         = -1;
        frames_received_since_last_key_ = kMinKeyFrameInterval;
        first_frame_                    = true;
        fps                             = std::min<int32_t>(kMaxObsVideoEncoderFps, fps);

        input_frame_infos_.clear();

        if (!WinObsAdapter::reset_video(width, height, fps)) {
            ret = -1;
            break;
        }

        if (!open()) {
            ret = -1;
            break;
        }

        if (!start()) {
            ret = -1;
            break;
        }
    } while (0);
    return ret;
}

int32_t WinObsVideoEncoder::register_encode_complete_callback_on_codec_thread( webrtc::EncodedImageCallback* callback) {
    callback_ = callback;
    return 0;
}

int32_t WinObsVideoEncoder::release_on_codec_thread() {
    if (started) {
        stop();
    }

    if (opened) {
        close();
    }

    return 0;
}

int32_t WinObsVideoEncoder::encode_on_codec_thread(
    const webrtc::VideoFrame& frame,
    const webrtc::FrameType frame_type) {
    struct encoder_frame  enc_frame;
    memset(&enc_frame, 0, sizeof(struct encoder_frame));

    //Check if encoder supports I420 input, otherwise, need to convert from I420 to NV12.
    if (preferred_video_format_ == VIDEO_FORMAT_I420) {
        enc_frame.data[0]       = (uint8_t *)frame.video_frame_buffer()->GetI420()->DataY();
        enc_frame.data[1]       = (uint8_t *)frame.video_frame_buffer()->GetI420()->DataU();
        enc_frame.data[2]       = (uint8_t *)frame.video_frame_buffer()->GetI420()->DataV();

        enc_frame.linesize[0]   = frame.video_frame_buffer()->GetI420()->StrideY();
        enc_frame.linesize[1]   = frame.video_frame_buffer()->GetI420()->StrideU();
        enc_frame.linesize[2]   = frame.video_frame_buffer()->GetI420()->StrideV();
    } else { //NV12
        webrtc::ConvertFromI420(frame, webrtc::VideoType::kNV12, 0, nv12_buff_.get());
        enc_frame.data[0]       = nv12_y_plane_;
        enc_frame.data[1]       = nv12_uv_plane_;
        enc_frame.linesize[0]   = width_;
        enc_frame.linesize[1]   = width_;
    }

    struct obs_encoder *encoder = h264_encoder_;
    if (!encoder->start_ts) {
        encoder->start_ts = frame.timestamp();
    }

    enc_frame.frames    = 1;
    enc_frame.pts       = encoder->cur_pts;
    bool req_key_frame  = (frame_type != webrtc::kVideoFrameDelta);

    int32_t ret = encode_thread_->Invoke<int32_t>(
        RTC_FROM_HERE,
        rtc::Bind(
            &WinObsVideoEncoder::do_encode, 
            this, 
            &enc_frame, 
            req_key_frame, 
            frame.timestamp(), 
            frame.render_time_ms()));

    encoder->cur_pts += encoder->timebase_num;

    return ret;
}

int32_t WinObsVideoEncoder::do_encode(
    struct encoder_frame *frame, 
    bool request_key_frame,
    int32_t timestamp,
    int64_t render_time_ms) {
    int ret = 0;
    struct obs_encoder *encoder = h264_encoder_;
    if (!encoder->profile_encoder_encode_name) {
        encoder->profile_encoder_encode_name = profile_store_name(obs_get_profiler_name_store(), "encode(%s)", encoder->context.name);
    }

    /************************************************************************/
    /*Key frame checking.                                                   */
    /************************************************************************/
    bool send_key_frame = false;
    ++frames_received_since_last_key_;
    int64_t now_ms = rtc::TimeMillis();
    if (last_frame_received_ms_ != -1 &&
        (now_ms - last_frame_received_ms_) > kFrameDiffThresholdMs) {
        if (frames_received_since_last_key_ > kMinKeyFrameInterval) {
            send_key_frame = true;
        }
        frames_received_since_last_key_ = 0;
    }
    last_frame_received_ms_ = now_ms;

    if (first_frame_) {
        send_key_frame = true;
        first_frame_ = false;
    }

    send_key_frame = request_key_frame || send_key_frame;
    
    //Store timestamp info.
    input_frame_infos_.emplace_back(timestamp, render_time_ms);

    struct encoder_packet pkt = { 0 };
    bool received       = false;
    bool success        = false;
    pkt.timebase_num    = encoder->timebase_num;
    pkt.timebase_den    = encoder->timebase_den;
    pkt.encoder         = encoder;
    pkt.keyframe        = send_key_frame;
    pkt.force_keyframe  = true;

    success = encoder->info.encode(encoder->context.data, frame, &pkt, &received);
    if (!success) {
        RTC_LOG(LS_ERROR) << "Error encoding with encoder " << encoder->context.name;
        ret = -1;
        input_frame_infos_.pop_back();
        goto error;
    }

    if (received) {
        if (!encoder->first_received) {
            encoder->offset_usec    = packet_dts_usec(&pkt);
            encoder->first_received = true;
        }

        /* we use system time here to ensure sync with other encoders,
        * you do not want to use relative timestamps here */
        pkt.dts_usec        = encoder->start_ts / 1000 + packet_dts_usec(&pkt) - encoder->offset_usec;
        pkt.sys_dts_usec    = pkt.dts_usec;

        pthread_mutex_lock(&encoder->callbacks_mutex);

        for (size_t i = encoder->callbacks.num; i > 0; i--) {
            struct encoder_callback *cb;
            cb = encoder->callbacks.array + (i - 1);
            send_packet(cb, &pkt);
        }

        pthread_mutex_unlock(&encoder->callbacks_mutex);
    }

error:
    return ret;
}

void WinObsVideoEncoder::send_packet(struct encoder_callback *cb, struct encoder_packet *packet) {
    struct obs_encoder *encoder = h264_encoder_;
    /* include SEI in first video packet */
    if (encoder->info.type == OBS_ENCODER_VIDEO && !cb->sent_first_packet) {
        send_first_video_packet(cb, packet);
    } else if (packet->keyframe) {
        //Add by HeZhen, WebRTC limit:Every key frame must begin with sps/pps.
        send_idr_packet(cb, packet);
    } else {
        cb->new_packet(cb->param, packet);
    }
}

void WinObsVideoEncoder::send_first_video_packet(
    struct encoder_callback *cb,
    struct encoder_packet *packet) {
    struct encoder_packet first_packet;
    DARRAY(uint8_t)       data;
    uint8_t               *sei;
    size_t                size;

    /* always wait for first keyframe */
    if (!packet->keyframe) {
        return;
    }

    da_init(data);

    struct obs_encoder *encoder = h264_encoder_;

    //Add sps/pps first, modified by HeZhen.
    uint8_t *header = NULL;
    obs_encoder_get_extra_data(encoder, &header, &size);
    da_push_back_array(data, header, size);

    if (!get_sei(&sei, &size) || !sei || !size) {
        cb->new_packet(cb->param, packet);
        cb->sent_first_packet = true;
        return;
    }

    da_push_back_array(data, sei, size);
    da_push_back_array(data, packet->data, packet->size);

    first_packet        = *packet;
    first_packet.data   = data.array;
    first_packet.size   = data.num;

    cb->new_packet(cb->param, &first_packet);
    cb->sent_first_packet = true;

    da_free(data);
}

void WinObsVideoEncoder::send_idr_packet(
    struct encoder_callback *cb,
    struct encoder_packet *packet) {
    struct obs_encoder *encoder = h264_encoder_;
    DARRAY(uint8_t) data;
    da_init(data);
    uint8_t *header;
    size_t size;
    obs_encoder_get_extra_data(encoder, &header, &size);
    da_push_back_array(data, header, size);
    da_push_back_array(data, packet->data, packet->size);

    struct encoder_packet idr_packet;
    idr_packet = *packet;
    idr_packet.data = data.array;
    idr_packet.size = data.num;

    cb->new_packet(cb->param, &idr_packet);

    da_free(data);
}

bool WinObsVideoEncoder::get_sei(
    uint8_t **sei,
    size_t *size) {
    struct obs_encoder *encoder = h264_encoder_;
    if (encoder->info.get_sei_data) {
        return encoder->info.get_sei_data(encoder->context.data, sei, size);
    } else {
        return false;
    }
}

///////////////////////////////////WinObsVideoEncoderFactory///////////////////////////////////////
WinObsVideoEncoderFactory::WinObsVideoEncoderFactory() {
    cricket::VideoCodec codec(cricket::kH264CodecName);
    const webrtc::H264::ProfileLevelId profile(webrtc::H264::kProfileConstrainedBaseline, webrtc::H264::kLevel3_1);
    std::string s = *webrtc::H264::ProfileLevelIdToString(profile);
    codec.SetParam(cricket::kH264FmtpProfileLevelId, *webrtc::H264::ProfileLevelIdToString(profile));
    codec.SetParam(cricket::kH264FmtpLevelAsymmetryAllowed, "1");
    codec.SetParam(cricket::kH264FmtpPacketizationMode, "1");
    supported_codecs_.push_back(codec);
}

WinObsVideoEncoderFactory::~WinObsVideoEncoderFactory() {

}

webrtc::VideoEncoder* WinObsVideoEncoderFactory::CreateVideoEncoder(const cricket::VideoCodec& codec) {
    return new WinObsVideoEncoder();
}

const std::vector<cricket::VideoCodec>& WinObsVideoEncoderFactory::supported_codecs() const {
    printf("111");
    return supported_codecs_;
}

void WinObsVideoEncoderFactory::DestroyVideoEncoder(webrtc::VideoEncoder* encoder) {
    delete encoder;
}

} // namespace bee

#endif // #ifdef OBS_ENCODER
