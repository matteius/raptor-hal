/* SPDX-License-Identifier: MIT */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "hal_h264_annexb.h"
#ifdef TEST_VUI
#include "rss_vui.h"
#endif

static void check_bytes(const uint8_t *packet, uint32_t length,
                        const rss_nal_unit_t *nals, uint32_t count)
{
    uint32_t offset = 0;
    for (uint32_t i = 0; i < count; ++i) {
        assert(nals[i].data == packet + offset);
        assert(nals[i].length <= length - offset);
        assert(nals[i].frame_end == (i + 1 == count));
        offset += nals[i].length;
    }
    assert(offset == length);
}

int main(void)
{
    /* Mixed 3/4-byte prefixes, SPS/PPS/SEI/AUD, multi-slice IDR, escaped
     * payload and trailing zeros. These are parser fixtures, not images. */
    const uint8_t packet[] = {
        0,0,0,1,0x67,0x64,0,0,3,1,0x80,
        0,0,1,0x68,0x80, 0,0,0,1,0x06,0x80,
        0,0,1,0x09,0x80, 0,0,0,1,0x65,0x88,
        0,0,1,0x65,0x88,0,0
    };
    const rss_nal_type_t types[] = {RSS_NAL_H264_SPS, RSS_NAL_H264_PPS,
        RSS_NAL_H264_SEI, RSS_NAL_UNKNOWN, RSS_NAL_H264_IDR, RSS_NAL_H264_IDR};
    rss_nal_unit_t nals[8];
    uint32_t count = 99;
    int key = 99;
    assert(hal_h264_annexb_split(packet, sizeof(packet), NULL, 0, &count, &key) == -ENOSPC);
    assert(count == 6 && key == 1);
    assert(hal_h264_annexb_split(packet, sizeof(packet), nals, 5, &count, &key) == -ENOSPC);
    assert(count == 6);
    assert(!hal_h264_annexb_split(packet, sizeof(packet), nals, 8, &count, &key));
    for (uint32_t i = 0; i < count; ++i)
        assert(nals[i].type == types[i]);
    check_bytes(packet, sizeof(packet), nals, count);

    const uint8_t p[] = {0,0,0,0,1,0x41,0x88,0,0};
    assert(!hal_h264_annexb_split(p, sizeof(p), nals, 8, &count, &key));
    assert(count == 1 && key == 0 && nals[0].type == RSS_NAL_H264_SLICE);
    check_bytes(p, sizeof(p), nals, count);

    const uint8_t invalid[][10] = {
        {0,0,0,2,0x65,0x88}, /* length prefixed */
        {0,0,1,0x67,0x80}, /* missing VCL */
        {0,0,1,0xe5,0x80}, /* forbidden zero bit */
        {1,0,0,1,0x65,0x80}, /* garbage before prefix */
        {0,0,1,0,0,1,0x65,0x80}, /* empty first NAL */
        {0,0,1,0x65,0x80,0,0,0,0,1} /* trailing empty NAL */
    };
    for (size_t i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i) {
        count = 99; key = 99;
        assert(hal_h264_annexb_split(invalid[i], sizeof(invalid[i]), nals, 8,
                                     &count, &key) == -EPROTO);
        assert(count == 99 && key == 99);
    }
    assert(hal_h264_annexb_split(NULL, 10, nals, 8, &count, &key) == -EINVAL);
    assert(hal_h264_annexb_split(packet, 0, nals, 8, &count, &key) == -EINVAL);
    assert(hal_h264_annexb_split(packet, sizeof(packet), NULL, 8, &count, &key) == -EINVAL);

    /* All truncations, including a prefix at the end, must remain bounded. */
    for (uint32_t length = 1; length < sizeof(packet); ++length) {
        int rc = hal_h264_annexb_split(packet, length, nals, 8, &count, &key);
        if (!rc)
            check_bytes(packet, length, nals, count);
        else
            assert(rc == -EPROTO);
    }
#ifdef TEST_VUI
    /* Same H.264 SPS fixture as raptor/tests/test_vui.c. Exercise the exact
     * SPS dispatch RVD uses; a single aggregate IDR descriptor skips it. */
    uint8_t au[] = {0,0,0,1,
        0x27,0x64,0x00,0x33,0xad,0x00,0xce,0x80,0x28,0x00,
        0xb5,0xa6,0xa0,0x20,0x20,0x3e,0x00,0x00,0x03,0x00,
        0x02,0x00,0x00,0x03,0x00,0x78,0x60,0x40,0x00,0x2d,
        0xc6,0xc0,0x00,0x11,0x2a,0x8f,0xff,0xf8,0x14,
        0,0,0,1,0x68,0x80, 0,0,0,1,0x65,0x88};
    assert(!hal_h264_annexb_split(au, sizeof(au), nals, 8, &count, &key));
    assert(count == 3 && key == 1 && nals[0].type == RSS_NAL_H264_SPS);
    uint8_t tail[12];
    memcpy(tail, au + sizeof(au) - sizeof(tail), sizeof(tail));
    for (uint32_t i = 0; i < count; ++i) {
        if (nals[i].type != RSS_NAL_H264_SPS)
            continue;
        assert(rss_vui_set_full_range((uint8_t *)nals[i].data, nals[i].length, 0) == 1);
        assert(rss_vui_set_matrix((uint8_t *)nals[i].data, nals[i].length, 0, 6) == 1);
        assert(rss_vui_set_full_range((uint8_t *)nals[i].data, nals[i].length, 0) == 0);
        assert(rss_vui_set_matrix((uint8_t *)nals[i].data, nals[i].length, 0, 6) == 0);
    }
    assert(!memcmp(tail, au + sizeof(au) - sizeof(tail), sizeof(tail)));
    check_bytes(au, sizeof(au), nals, count);
#endif
    puts("H.264 Annex-B descriptors: PASS");
    return 0;
}
