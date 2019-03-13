#ifdef OBS_ENCODER

#include "examples/peerconnection/client/win_obs_adapter.h"

extern "C" {
#include "libobs/obs-internal.h"
}

#include "rtc_base/logging.h"

namespace webrtc {

bool        WinObsAdapter::initialized_         = false;
ConfigFile  WinObsAdapter::basic_config_;
std::string WinObsAdapter::basic_config_path_   = "obs.ini";
std::string WinObsAdapter::plugin_config_path_  = "obs_plugin_config";

extern "C" void obs_log_callback(int32_t log_level, const char *format, va_list args, void *param) {
    char out[4096];
    vsnprintf(out, sizeof(out), format, args);
    switch (log_level) {
    case LOG_ERROR:
        RTC_LOG(LS_ERROR) << out;
        break;
    case LOG_WARNING:
        RTC_LOG(LS_WARNING) << out;
        break;
    case LOG_INFO:
        RTC_LOG(LS_INFO) << out;
        break;
    case LOG_DEBUG:
    default:
        RTC_LOG(LS_VERBOSE) << out;
    }
}

bool WinObsAdapter::init() {
    bool ret = true;
    do {
        if (initialized_) {
            break;
        }

        //Hook obs logging.
        base_set_log_handler(obs_log_callback, NULL);

        //Start up obs, create obs global object.
        ret = obs_startup("en-US", plugin_config_path_.c_str(), NULL);
        if (!ret) {
            break;
        }

        //Load config.
        ret = init_basic_config();
        if (!ret) {
            break;
        }

        //Load custom set, bypass graphic module.
        obs_customize_set();

        //Load plugins.
        obs_load_all_modules();

        //Logging loaded plugins.
        obs_log_loaded_modules();

        initialized_ = true;
    } while (0);
    return ret;
}

void WinObsAdapter::uninit() {
    do {
        if (!initialized_) {
            break;
        }

        obs_shutdown();
    } while (0);
}

ConfigFile& WinObsAdapter::get_config() {
    return basic_config_;
}

bool WinObsAdapter::reset_video(uint32_t width, uint32_t height, uint32_t fps) {
    struct obs_video_info ovi;

    ovi.fps_num             = fps;
    ovi.fps_den             = 1;

    const char *colorFormat = config_get_string(basic_config_, "Video", "ColorFormat");
    const char *colorSpace  = config_get_string(basic_config_, "Video", "ColorSpace");
    const char *colorRange  = config_get_string(basic_config_, "Video", "ColorRange");

    ovi.base_width          = width;
    ovi.base_height         = height;
    ovi.output_width        = width;
    ovi.output_height       = height;
    ovi.output_format       = get_video_format_from_name(colorFormat);
    ovi.colorspace          = astrcmpi(colorSpace, "601") == 0 ? VIDEO_CS_601 : VIDEO_CS_709;
    ovi.range               = astrcmpi(colorRange, "Full") == 0 ? VIDEO_RANGE_FULL : VIDEO_RANGE_PARTIAL;
    ovi.gpu_conversion      = true;
    ovi.scale_type          = get_scale_type();

    if (ovi.base_width == 0 || ovi.base_height == 0) {
        ovi.base_width      = 1920;
        ovi.base_height     = 1080;
        config_set_uint(basic_config_, "Video", "BaseCX", 1920);
        config_set_uint(basic_config_, "Video", "BaseCY", 1080);
    }

    if (ovi.output_width == 0 || ovi.output_height == 0) {
        ovi.output_width    = ovi.base_width;
        ovi.output_height   = ovi.base_height;
        config_set_uint(basic_config_, "Video", "OutputCX", ovi.base_width);
        config_set_uint(basic_config_, "Video", "OutputCY", ovi.base_height);
    }

    int32_t ret = obs_reset_video(&ovi);
    return ret == 0;
}

bool WinObsAdapter::init_basic_config() {
    int32_t code = basic_config_.Open(basic_config_path_.c_str(), CONFIG_OPEN_ALWAYS);
    return code == 0;
}

enum video_format WinObsAdapter::get_video_format_from_name(const char *name) {
    if (name == NULL) {
        return VIDEO_FORMAT_NONE;
    }

    if (astrcmpi(name, "I420") == 0) {
        return VIDEO_FORMAT_I420;
    } else if (astrcmpi(name, "NV12") == 0) {
        return VIDEO_FORMAT_NV12;
    } else if (astrcmpi(name, "I444") == 0) {
        return VIDEO_FORMAT_I444;
    }
#if 0 //currently unsupported
    else if (astrcmpi(name, "YVYU") == 0) {
        return VIDEO_FORMAT_YVYU;
    } else if (astrcmpi(name, "YUY2") == 0) {
        return VIDEO_FORMAT_YUY2;
    } else if (astrcmpi(name, "UYVY") == 0) {
        return VIDEO_FORMAT_UYVY;
    }
#endif
    else {
        return VIDEO_FORMAT_RGBA;
    }
}

enum obs_scale_type WinObsAdapter::get_scale_type() {
    const char *scaleTypeStr = config_get_string(basic_config_, "Video", "ScaleType");
    if (astrcmpi(scaleTypeStr, "bilinear") == 0) {
        return OBS_SCALE_BILINEAR;
    } else if (astrcmpi(scaleTypeStr, "lanczos") == 0) {
        return OBS_SCALE_LANCZOS;
    } else {
        return OBS_SCALE_BICUBIC;
    }
}

} // namespace bee
#endif // #ifdef OBS_ENCODER
