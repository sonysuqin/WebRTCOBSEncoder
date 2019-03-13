# 1 Purpose
Under windows, WebRTC uses OpenH264 for encoding H264. OpenH264 is a software encoder, which make use of cpu for encoding. If you publish your WebRTC stream with very high resolution like 1080P, you may think about hardware acceleration because OpenH264 will cause very high cpu usage, certainly other software encoder will make the nearly same result.

OBS studio is a very popular stable open source live streaming software, with some H264 encoder integrated, such as

- obs_x264, software;
- INTEL obs_qsv11, hardware;
- AMD amd_amf_h264, hardware;
- NVIDIA ffmpeg_nvenc, hardware.

With these encoders, we can design WebRTC real-time products supporting high resolution with low cpu usage under Windows, this article and source code will introduce detailly how to achieve this goal.

# 2 Code 
[https://github.com/sonysuqin/WebRTCOBSEncoder](https://github.com/sonysuqin/WebRTCOBSEncoder)

# 3 Version
| Software | Version |
|:--|:--|
| WebRTC | M66 |
| OBS | 21.0.3 |

# 4 Environment
| Software | Remark |
|:--|:--|
| Windows 10 |  |
| Visual studio 2017 | With foundation C++, MFC, Windows 10 SDK. |
| Linux  subsystem| By installing Windows10 ubuntu subsystem, or other software like Cygwin, MinGW, etc.|
| git | Under Linux subsystem. |

# 5 Build
## 5.1 Directory Structure
Make sure the directory structure is:
```
|-- obs
|-- WebRTC
`-- WebRTCOBSEncoder
```

## 5.2 Clone WebRTCOBSEncoder
```
git clone https://github.com/sonysuqin/WebRTCOBSEncoder.git
```

## 5.3 Build WebRTC
### 5.3.1 How to build WebRTC
Follow the [official tutorial](https://webrtc.org/native-code/development/).
The most important point is that you should make the 3 components bellow with the same version, or compatible, just make them all newest.
- WebRTC code(You should use the latest stable branch, not the developing branch);
- Windows 10 SDK(You should be demanded to install "Debugging Tools for Windows" manually);
- depot_tool.

## 5.3.2 Modify "peerconnection/client" demo
After you cloned WebRTCOBSEncoder's code, you can see example source code structure

```
src
|-- example
|   `-- peerconnection
|       `-- client
|           |-- conductor.cc
|           |-- conductor.h
|           |-- defaults.cc
|           |-- defaults.h
|           |-- flagdefs.h
|           |-- linux
|           |   |-- main.cc
|           |   |-- main_wnd.cc
|           |   `-- main_wnd.h
|           |-- main.cc
|           |-- main_wnd.cc
|           |-- main_wnd.h
|           |-- peer_connection_client.cc
|           |-- peer_connection_client.h
|           |-- win_obs_adapter.cpp
|           |-- win_obs_adapter.h
|           |-- win_obs_video_encoder.cpp
|           `-- win_obs_video_encoder.h
`-- out
    `-- win32
        `-- obj
            `-- examples
                `-- peerconnection_client.ninja
```
It modifies the WebRTC's peerconnection/client demo, and with exactly the same structure, now copy and overriden all these files from WebRTCOBSEncoder to WebRTC's directory in the same location.

By type command "git diff" you can see what modified, how to invoke wrapped obs encoder.

The main obs encoder implmentation files list below:

```
|-- win_obs_adapter.cpp
|-- win_obs_adapter.h
|-- win_obs_video_encoder.cpp
`-- win_obs_video_encoder.h
```

## 5.4 Build OBS
### 5.4.1 Clone OBS
Make sure OBS directory is in the same level as WebRTCOBSEncoder's directory.
```
git clone https://github.com/obsproject/obs-studio.git obs
```

### 5.4.2 Checkout branch 21.0.3
```
cd obs
git branch my_branch 21.0.3
git checkout my_branch
```
> You can use another branch, but the patch won't work correctly, if you have to do so, you have to manually check all modifies in patch and move them to your branch.

### 5.4.3 Patch
```
cd patch
./patch.sh
```
> Only working for 21.0.3 branch.

### 5.4.4 How to build OBS
Follow the [official tutorial](https://github.com/obsproject/obs-studio/wiki/Install-Instructions).

### 5.4.5 Copy OBS headers
Make a standalone obs header copy, there's a tool under WebRTCOBSEncoder:
```
script/
`-- cph.sh

cph.sh obs WebRTC/src/third_party/obs/win32/include
```
> Or you can set your obs header search path to obs directory by manually modify peerconnection_client.ninja.

### 5.4.6 Copy obs libs
Make a standalone obs encoder libs copy,  you should copy the following libs:

```
mkdir -p WebRTC/src/third_party/obs/win32/lib
cp obs/build/libobs/RelWithDebInfo/obs.lib WebRTC/src/third_party/obs/win32/lib/
cp obs/build/rundir/RelWithDebInfo/bin/32bit/w32-pthreads.lib WebRTC/src/third_party/obs/win32/lib/
```

### 5.4.7 Copy obs depending dlls
Copy following dlls to your executable path.
```
obs/build/rundir/RelWithDebInfo/bin/32bit/avcodec-57.dll
obs/build/rundir/RelWithDebInfo/bin/32bit/avdevice-57.dll
obs/build/rundir/RelWithDebInfo/bin/32bit/avformat-57.dll
obs/build/rundir/RelWithDebInfo/bin/32bit/avutil-55.dll
obs/build/rundir/RelWithDebInfo/bin/32bit/libx264-148.dll
obs/build/libobs/RelWithDebInfo/obs.dll
obs/build/rundir/RelWithDebInfo/bin/32bit/w32-pthreads.dll
obs/build/rundir/RelWithDebInfo/bin/32bit/zlib.dll
```

### 5.4.8 Copy obs encoder plugin dlls
Copy the following directory to your executable path.

```
obs/build/rundir/RelWithDebInfo/obs-plugins
```
Keep the following plugins, you can delete others.

```
enc-amf.dll
obs-ffmpeg.dll
obs-qsv11.dll
obs-x264.dll
win-mf.dll
```

### 5.4.9 Copy obs.ini
Copy the following file to your executable path.
```
config/
`-- obs.ini
```
This file is almost the same as original OBS configure by which you can set all OBS supported codec parameters.

## 5.5 Rebuild WebRTC "peerconnection/client" Demo
Just rebuilt it from IDE or from shell by ninja command.

# 6 Test
Under WebRTC output executable path:
- Run one peerconnection_server.exe;
- Run two peerconnection_client.exe, both connect to the same peerconnection_server.exe;
- Click the target in the user list to make a end-to-end streaming, both of the clients should be in the same LAN.