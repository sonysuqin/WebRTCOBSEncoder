#ifdef OBS_ENCODER
#ifndef __WIN_OBS_ADAPTER_H__
#define __WIN_OBS_ADAPTER_H__

#include "libobs/obs.h"
#include "libobs/util/util.hpp"

#include <string>

namespace webrtc {

class WinObsAdapter {
public:
    static bool init();
    static void uninit();
    static ConfigFile& get_config();
    static bool reset_video(uint32_t width, uint32_t height, uint32_t fps);

private:
    static bool init_basic_config();
    static enum video_format get_video_format_from_name(const char *name);
    static enum obs_scale_type get_scale_type();

private:
    static bool         initialized_;
    static ConfigFile   basic_config_;
    static std::string  basic_config_path_;
    static std::string  plugin_config_path_;
};

} // namespace bee

#endif // #ifdef __WIN_OBS_ADAPTER_H__
#endif // #ifdef OBS_ENCODER
