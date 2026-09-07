#include <assert.h>
#include "hal_internal.h"

int hal_isp_get_exposure(void *ctx, rss_exposure_t *exposure);

static IMPISPAEIntegrationTimeUnit returned_unit;
static int query_result;

static void test_log(int level, const char *file, int line,
                     const char *format, ...)
{
    (void)level;
    (void)file;
    (void)line;
    (void)format;
}

rss_hal_log_func_t rss_hal_log_fn = test_log;

int32_t IMP_ISP_Tuning_GetAeExprInfo(IMPVI_NUM num,
                                  IMPISPAeExprInfo *info)
{
    assert(num == IMPVI_MAIN);
    if (query_result)
        return query_result;
    memset(info, 0, sizeof(*info));
    info->AeIntegrationTimeUnit = returned_unit;
    info->AeIntegrationTime = 23;
    info->AeAGain = 1024;
    return 0;
}

int32_t IMP_ISP_Tuning_GetAeStatistics(IMPVI_NUM num,
                                    IMPISPAEStatisInfo *info)
{
    assert(num == IMPVI_MAIN);
    memset(info, 0, sizeof(*info));
    info->ae_hist_256bin[70] = 100;
    return 0;
}

int32_t IMP_ISP_Tuning_GetAwbGlobalStatistics(IMPVI_NUM num,
                                            IMPISPAWBGlobalStatisInfo *info)
{
    assert(num == IMPVI_MAIN);
    memset(info, 0, sizeof(*info));
    return 0;
}

int main(void)
{
    rss_exposure_t exposure;

    returned_unit = ISP_CORE_EXPR_UNIT_US;
    assert(hal_isp_get_exposure(NULL, &exposure) == RSS_OK);
    assert(exposure.valid_mask & RSS_EXPOSURE_VALID_TIME);
    assert(exposure.exposure_time == 23);

    returned_unit = ISP_CORE_EXPR_UNIT_LINE;
    assert(hal_isp_get_exposure(NULL, &exposure) == RSS_OK);
    assert(!(exposure.valid_mask & RSS_EXPOSURE_VALID_TIME));
    assert(exposure.exposure_time == 0);
    assert(exposure.valid_mask & RSS_EXPOSURE_VALID_TOTAL_GAIN);
    assert(exposure.valid_mask & RSS_EXPOSURE_VALID_AE_LUMA);
    assert(exposure.ae_luma == 70);

    returned_unit = (IMPISPAEIntegrationTimeUnit)99;
    assert(hal_isp_get_exposure(NULL, &exposure) == RSS_OK);
    assert(!(exposure.valid_mask & RSS_EXPOSURE_VALID_TIME));

    query_result = -1;
    assert(hal_isp_get_exposure(NULL, &exposure) == -1);
    assert(exposure.valid_mask == 0);
    assert(hal_isp_get_exposure(NULL, NULL) == RSS_ERR_INVAL);
    return 0;
}
