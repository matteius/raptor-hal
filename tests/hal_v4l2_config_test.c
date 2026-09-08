/* SPDX-License-Identifier: MIT */
/* Exercise the actual adapter's private config builder; unused hardware
 * entry points are discarded by the linker's section GC. No device needed. */
#include "../src/hal_v4l2.c"
#include <assert.h>

int main(void)
{
    rss_video_config_t config = {
        .codec = RSS_CODEC_H264, .width = 1280, .height = 720,
        .fps_num = 30000, .fps_den = 1001, .bitrate = 2000000,
        .profile = 2, .rc_mode = RSS_RC_CBR,
    };
    OpenIMPAVCConfig avc = v4l2_avc_config(&config);
    assert(RSS_V4L2_BUFFER_COUNT == 2);
    assert(avc.stream_buffer_count == 1 && avc.stream_buffer_size == 0);
    assert(avc.width == 1280 && avc.height == 720 && avc.bitrate == 2000000);
    assert(avc.fps_num == 30000 && avc.fps_den == 1001);
    assert(avc.min_qp == 15 && avc.max_qp == 45 && avc.initial_qp == 26);
    config.max_stream_cnt = 4;
    config.stream_buf_size = 1048576;
    config.gop_length = 60;
    config.min_qp = 30;
    avc = v4l2_avc_config(&config);
    assert(avc.stream_buffer_count == 4 && avc.stream_buffer_size == 1048576);
    assert(avc.gop_length == 60 && avc.initial_qp == 30);
    config.init_qp = 50;
    avc = v4l2_avc_config(&config);
    assert(avc.initial_qp == 45);
    puts("V4L2 serial output-pool configuration passed");
    return 0;
}
