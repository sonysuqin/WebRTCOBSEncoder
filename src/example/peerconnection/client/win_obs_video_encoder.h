/**
*  @file        win_obs_video_encoder.h
*  @brief       OBS编码器声明文件.
*  @author      sonysuqin
*  @copyright   sonysuqin
*  @version     1.0.0.1
*/

#ifdef OBS_ENCODER
#ifndef __WIN_OBS_VIDEO_ENCODER_H__
#define __WIN_OBS_VIDEO_ENCODER_H__

#include "libobs/obs.hpp"
#include "libobs/util/util.hpp"

extern "C" {
#include "libobs/obs-internal.h"
}

#include "api/video_codecs/video_encoder.h"
#include "rtc_base/asyncinvoker.h"
#include "media/engine/webrtcvideoencoderfactory.h"
#include "common_video/h264/h264_bitstream_parser.h"
#include "system_wrappers/include/clock.h"

namespace webrtc {

const size_t kFrameDiffThresholdMs  = 350;      //!< 无数据到达的最大时间阈值.
const size_t kMinKeyFrameInterval   = 6;        //!< 最小关键帧间隔阈值.

/// OBS编码器类
class WinObsVideoEncoder : public webrtc::VideoEncoder {
public:
    /**
    *  @brief  WinObsVideoEncoder类构造函数.
    */
    WinObsVideoEncoder();

    /**
    *  @brief  WinObsVideoEncoder类析构函数.
    */ 
    virtual ~WinObsVideoEncoder();

public:
    /**
    *  @brief  WebRTC初始化编码器回调.
    *  @param  codec_settings       编码器配置，目前无用，主要以构造函数传入的OBS配置为准.
    *  @param  number_of_cores      可用核心数，目前无用.
    *  @param  max_payload_size     最大载荷大小，目前无用.
    *  @return 错误码.
    */
    virtual int32_t InitEncode(
        const webrtc::VideoCodec* codec_settings,
        int32_t number_of_cores,
        size_t max_payload_size) override;

    /**
    *  @brief  WebRTC注册编码完成回调的回调.
    *  @param  callback             编码完成回调，编码后的帧通过该回调传给WebRTC.
    *  @return 错误码.
    */
    virtual int32_t RegisterEncodeCompleteCallback(
        webrtc::EncodedImageCallback* callback) override;

    /**
    *  @brief  WebRTC释放编码器回调.
    *  @return 错误码.
    */
    virtual int32_t Release() override;

    /**
    *  @brief  WebRTC编码回调.
    *  @param  frame                待编码帧.
    *  @param  codec_specific_info  编码信息，目前未用.
    *  @param  frame_types          帧类型, 关键帧/非关键帧.
    *  @return 错误码.
    */
    virtual int32_t Encode(
        const webrtc::VideoFrame& frame,
        const webrtc::CodecSpecificInfo* codec_specific_info,
        const std::vector<webrtc::FrameType>* frame_types) override;

    /**
    *  @brief  WebRTC通知通道参数回调.
    *  @param  packet_loss          丢包率.
    *  @param  rtt                  rtt.
    *  @note   理论上应该根据这两个参数来重置编码器，目前未用.
    *  @return 错误码.
    */
    virtual int32_t SetChannelParameters(
        uint32_t packet_loss,
        int64_t rtt) override;

    /**
    *  @brief  WebRTC通知码率、帧率回调.
    *  @param  rate_allocation      码率.
    *  @param  frame_rate           帧率.
    *  @note   理论上应该根据这两个参数来重置编码器，目前未用.
    *  @return 错误码.
    */
    virtual int32_t SetRateAllocation(
        const webrtc::BitrateAllocation& rate_allocation,
        uint32_t frame_rate) override;

private:
    /**
    *  @brief  打开OBS编码器.
    *  @return 成功/失败.
    */
    bool open();

    /**
    *  @brief  关闭OBS编码器.
    *  @return 成功/失败.
    */
    bool close();

    /**
    *  @brief  启动OBS编码器.
    *  @return 成功/失败.
    */
    bool start();

    /**
    *  @brief  停止OBS编码器.
    *  @return 成功/失败.
    */
    bool stop();

    /**
    *  @brief  内部启动OBS编码器.
    *  @return 成功/失败.
    */
    bool start_internal();

    /**
    *  @brief  内部停止OBS编码器.
    *  @return 成功/失败.
    */
    bool stop_internal();

    /**
    *  @brief  配置编码器.
    *  @param  config               配置文件.
    */
    void configure_encoder(const ConfigFile& config);

    /**
    *  @brief  OBS编码完成回调.
    *  @param  param                传入OBS编码器的回传参数(this).
    *  @param  packet               编码完成的包.
    */
    static void new_encoded_packet(void *param, struct encoder_packet *packet);

    /**
    *  @brief  处理OBS编码完成的包.
    *  @param  packet               编码完成的包.
    */
    void on_new_encoded_packet(struct encoder_packet *packet);

    /**
    *  @brief  加载指定的H264编码器.
    *  @param  encoderId            编码器名字.
    *  @return 成功/失败.
    */
    bool load_streaming_preset_h264(const char *encoderId);

    /**
    *  @brief  更新amd编码器的默认设置.
    *  @param  settings             H264设置对象.
    *  @param  bitrate              目标码率.
    */
    void update_streaming_settings_amd(obs_data_t *settings, int bitrate);

    /**
    *  @brief  获取编码器的回调对象索引.
    *  @param  encoder              OBS编码器对象.
    *  @param  new_packet           编码完成回调.
    *  @param  param                传入OBS编码器的回传参数(this).
    *  @return 回调对象索引.
    */
    size_t get_callback_idx(
        const struct obs_encoder *encoder,
        void(*new_packet)(void *param, struct encoder_packet *packet),
        void *param);

    /**
    *  @brief  实际释放OBS编码器.
    *  @param  encoder              OBS编码器对象.
    */
    void obs_encoder_actually_destroy(obs_encoder_t *encoder);

    /**
    *  @brief  在编码线程初始化编码器.
    *  @param  width                图像宽.
    *  @param  height               图像高.
    *  @param  target_bitrate       目标码率, 目前无用.
    *  @param  fps                  帧率， 目前无用.
    *  @return 错误码.
    */
    int32_t init_encode_on_codec_thread(int32_t width, int32_t height, int32_t target_bitrate, int32_t fps);

    /**
    *  @brief  在编码线程设置WebRTC编码完成回调.
    *  @param  callback             WebRTC编码完成回调.
    *  @return 错误码.
    */
    int32_t register_encode_complete_callback_on_codec_thread(webrtc::EncodedImageCallback* callback);

    /**
    *  @brief  在编码线程释放编码器.
    *  @return 错误码.
    */
    int32_t release_on_codec_thread();

    /**
    *  @brief  在编码线程调用编码.
    *  @param  frame                待编码帧.
    *  @param  frame_types          帧类型, 关键帧/非关键帧.
    *  @return 错误码.
    */
    int32_t encode_on_codec_thread(
        const webrtc::VideoFrame& frame,
        const webrtc::FrameType frame_type);

    /**
    *  @brief  OBS编码方法.
    *  @param  frame                待编码帧.
    *  @param  request_key_frame    是否请求编码关键帧.
    *  @param  timestamp            RTP时间戳，以1/90ms为单位.
    *  @param  render_time_ms       显示时间戳.
    *  @return 错误码.
    */
    int32_t do_encode(
        struct encoder_frame *frame, 
        bool request_key_frame, 
        int32_t timestamp, 
        int64_t render_time_ms);

    /**
    *  @brief  编码完成调用回调发送编码结果.
    *  @param  cb                   OBS编码完成回调.
    *  @param  packet               已编码完成的帧.
    *  @note   这里在发送前做额外的工作，例如添加SPS、PPS.
    */
    void send_packet(struct encoder_callback *cb, struct encoder_packet *packet);

    /**
    *  @brief  发送第一个包.
    *  @param  cb                   OBS编码完成回调.
    *  @param  packet               已编码完成的帧.
    */
    void send_first_video_packet(
        struct encoder_callback *cb,
        struct encoder_packet *packet);

    /**
    *  @brief  发送IDR帧，事实上WebRTC要求每个关键帧都是IDR帧.
    *  @param  cb                   OBS编码完成回调.
    *  @param  packet               已编码完成的帧.
    */
    void send_idr_packet(
        struct encoder_callback *cb,
        struct encoder_packet *packet);

    /**
    *  @brief  获取SEI包.
    *  @param  sei                  SEI包缓冲.
    *  @param  size                 缓冲长度.
    */
    bool get_sei(
        uint8_t **sei,
        size_t *size);

private:
    /// 输入帧信息，用于存储时间戳信息.
    struct InputFrameInfo {
        InputFrameInfo(
            int32_t frame_timestamp,
            int64_t frame_render_time_ms)
            : frame_timestamp(frame_timestamp),
              frame_render_time_ms(frame_render_time_ms) {}
        const int32_t frame_timestamp;
        const int64_t frame_render_time_ms;
    };

private:
    /// OBS编码器对象.
    OBSEncoder h264_encoder_;
    /// 编码器是否已打开.
    bool opened  = false;
    /// 编码器是否已启动.
    bool started = false;
    /// 输入图像宽.
    int32_t width_ = 0;
    /// 输入图像高.
    int32_t height_ = 0;

    /// 编码线程.
    std::unique_ptr<rtc::Thread> encode_thread_;
    /// WebRTC编码完成回调.
    webrtc::EncodedImageCallback* callback_ = NULL;
    /// WebRTC H264流解析器，主要用于获取QP.
    webrtc::H264BitstreamParser h264_bitstream_parser_;

    /// 输入帧的时间戳信息队列.
    std::list<InputFrameInfo> input_frame_infos_;
    /// RTP时间戳, 以1/90ms为单位.
    int32_t output_timestamp_ = 0;
    /// 相对时间戳，以ms为单位.
    int64_t output_render_time_ms_ = 0;

    /// 时钟.
    webrtc::Clock* clock_;
    /// NTP时间(1900)与UNIX时间(1970)的差值.
    int64_t delta_ntp_internal_ms_ = 0;

    /// 最后收到待编码帧的时间.
    int64_t last_frame_received_ms_ = 0;
    /// 距上次收到编码关键帧请求的时间.
    int32_t frames_received_since_last_key_ = 0;
    /// 是否第一帧.
    bool first_frame_ = true;

    /// 编码器输入图像格式.
    enum video_format preferred_video_format_ = VIDEO_FORMAT_NV12;

    /// NV12缓冲.
    std::unique_ptr<uint8_t[]> nv12_buff_;
    /// NV12缓冲长度.
    size_t  frame_length_ = 0;
    /// NV12 Y Plane指针.
    uint8_t *nv12_y_plane_ = NULL;
    /// NV12 UV Plane指针.
    uint8_t *nv12_uv_plane_ = NULL;
};

////////////////////////////////////WinObsVideoEncoderFactory//////////////////////////////////////
class WinObsVideoEncoderFactory : public cricket::WebRtcVideoEncoderFactory {
public:
    WinObsVideoEncoderFactory();
    ~WinObsVideoEncoderFactory();

public:
    /**
    *  @brief  创建WebRTC视频编码器.
    *  @param  codec                编码器信息.
    *  @return WebRTC视频编码器.
    */
    webrtc::VideoEncoder* CreateVideoEncoder(const cricket::VideoCodec& codec) override;

    /**
    *  @brief  获取支持的编码.
    *  @return 编码器支持的编码.
    */
    const std::vector<cricket::VideoCodec>& supported_codecs() const override;

    /**
    *  @brief  销毁WebRTC视频编码器.
    *  @param  encoder              WebRTC视频编码器.
    */
    void DestroyVideoEncoder(webrtc::VideoEncoder* encoder) override;

private:
    /// 编码器支持的编码.
    std::vector<cricket::VideoCodec> supported_codecs_;
};

} // namespace bee

#endif // #ifndef __WIN_OBS_VIDEO_ENCODER_H__
#endif // #ifdef OBS_ENCODER
