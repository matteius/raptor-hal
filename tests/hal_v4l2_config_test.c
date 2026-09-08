/* SPDX-License-Identifier: MIT */
/* Exercise the actual adapter's private config builder; unused hardware
 * entry points are discarded by the linker's section GC. No device needed. */
#include "../src/hal_v4l2.c"
#include <assert.h>

int OpenIMP_AVC_SetBitrate(OpenIMPAVCEncoder *encoder, uint32_t bitrate)
{
    (void)encoder;
    (void)bitrate;
    assert(!"control setter must defer the hardware call");
    return 0;
}

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
    {
        rss_hal_ctx_t ctx = {0};
        rss_v4l2_h264_t main = {0}, sub = {0};
        char path[64];
        ctx.v4l2[0] = &main;
        ctx.v4l2[1] = &sub;
        atomic_init(&main.pending_bitrate, 0);
        atomic_init(&sub.pending_bitrate, 0);
        assert(v4l2_channel_valid(0) && v4l2_channel_valid(1) && v4l2_channel_valid(2));
        assert(!v4l2_channel_valid(-1) && !v4l2_channel_valid(3));
        assert(!v4l2_ops_enc_set_bitrate(&ctx, 1, 123456));
        assert(atomic_load(&sub.pending_bitrate) == 123456);
        assert(atomic_load(&main.pending_bitrate) == 0);
        assert(v4l2_ops_enc_set_bitrate(&ctx, 2, 123456) == -EINVAL);
        assert(v4l2_ops_enc_set_bitrate(&ctx, -1, 123456) == -EINVAL);
        assert(v4l2_ops_enc_set_bitrate(&ctx, RSS_MAX_ENC_CHANNELS, 123456) == -EINVAL);
        assert(!v4l2_channel_device(&ctx, 0, path, sizeof(path)));
        assert(!strcmp(path, "/dev/video0"));
        rss_hal_v4l2_set_device(&ctx, "/dev/video17");
        assert(!v4l2_channel_device(&ctx, 0, path, sizeof(path)));
        assert(!strcmp(path, "/dev/video17"));
    }
    puts("V4L2 serial output-pool configuration passed");
    return 0;
}
