/* SPDX-License-Identifier: MIT
 * Zero-copy Annex-B access-unit descriptors. No sensor or encoder assumptions.
 */
#ifndef HAL_H264_ANNEXB_H
#define HAL_H264_ANNEXB_H

#include <errno.h>
#include "raptor_hal.h"

static rss_nal_type_t hal_h264_nal_type(uint8_t type)
{
    switch (type) {
    case 1: case 2: case 3: case 4: return RSS_NAL_H264_SLICE;
    case 5: return RSS_NAL_H264_IDR;
    case 6: return RSS_NAL_H264_SEI;
    case 7: return RSS_NAL_H264_SPS;
    case 8: return RSS_NAL_H264_PPS;
    default: return RSS_NAL_UNKNOWN;
    }
}

/* Keep start codes and every byte in the original access unit. In particular,
 * splitting must not change the ring payload or require a second video copy.
 * On ENOSPC, count reports the required descriptor capacity; partially filled
 * descriptors must not be published. Other failures leave count/key untouched.
 * The caller retains the packet until all descriptors have been consumed.
 */
static int hal_h264_annexb_split(const uint8_t *data, uint32_t length,
                                rss_nal_unit_t *nals, uint32_t capacity,
                                uint32_t *count, int *is_key)
{
    uint32_t i = 0, start = 0, header = 0, used = 0;
    int have_nal = 0, saw_vcl = 0, key = 0;
    rss_nal_type_t type = RSS_NAL_UNKNOWN;

    if (!data || !length || !count || !is_key || (capacity && !nals))
        return -EINVAL;
    while (i < length) {
        uint32_t prefix = 0;
        if (length - i >= 3 && data[i] == 0 && data[i + 1] == 0) {
            if (data[i + 2] == 1)
                prefix = 3;
            else if (length - i >= 4 && data[i + 2] == 0 && data[i + 3] == 1)
                prefix = 4;
        }
        if (!prefix) {
            if (!have_nal && data[i] != 0)
                return -EPROTO; /* not Annex-B, or nonzero leading garbage */
            ++i;
            continue;
        }
        if (length - i <= prefix || (data[i + prefix] & 0x80))
            return -EPROTO;
        if (have_nal) {
            if (i <= header)
                return -EPROTO;
            if (used < capacity)
                nals[used] = (rss_nal_unit_t){data + start, i - start, type, false};
            ++used;
            start = i;
        }
        header = i + prefix;
        uint8_t raw_type = data[header] & 0x1f;
        if (!raw_type)
            return -EPROTO;
        type = hal_h264_nal_type(raw_type);
        saw_vcl |= raw_type >= 1 && raw_type <= 5;
        key |= raw_type == 5;
        have_nal = 1;
        i = header + 1;
    }
    if (!have_nal || !saw_vcl)
        return -EPROTO;
    if (used < capacity)
        nals[used] = (rss_nal_unit_t){data + start, length - start, type, true};
    ++used;
    *count = used;
    *is_key = key;
    return used > capacity ? -ENOSPC : 0;
}

#endif
