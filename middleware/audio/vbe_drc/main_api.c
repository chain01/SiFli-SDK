#include  <rtthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <arm_math.h>

#include "vbe_eq_drc_api.h"
#include "eq_filter.h"
#include "crossover_3b.h"
#include "vbe_eq_drc_config.h"
#include "vbe_eq_drc_internal.h"

#define DBG_TAG           ""
#define DBG_LVL           LOG_LVL_INFO
#include "log.h"

#define vbe_malloc  rt_malloc
#define vbe_free    rt_free


//vbe eq drc load config
static void load_config(vbe_drc_t *thiz)
{
    thiz->should_update_config = 0;
    if ((low.enable == 0) && (mid.enable == 0) && (high.enable == 0))
    {
        thiz->drc_enable = 0;
    }
    else
    {
        thiz->drc_enable = 1;
    }

    thiz->drcParaLow.enable = low.enable;
    thiz->drcParaLow.compressorThreshold = low.compressor_threshhold;
    thiz->drcParaLow.compressorRatio = low.compressor_ratio;
    thiz->drcParaLow.compressorKneeWidth = low.compressor_kneewidth;
    thiz->drcParaLow.expanderThreshold = low.expander_threshhold;
    thiz->drcParaLow.expanderRatio = low.expander_ratio;
    thiz->drcParaLow.expanderKneeWidth = low.expander_kneewidth;
    thiz->drcParaLow.alphaA = low.alpha_a;
    thiz->drcParaLow.betaA = low.beta_a;
    thiz->drcParaLow.alphaR = low.alpha_r;
    thiz->drcParaLow.betaR = low.beta_r;
    thiz->drcParaLow.makeupGain = low.gain;

    thiz->drcParaMid.enable = mid.enable;
    thiz->drcParaMid.compressorThreshold = mid.compressor_threshhold;
    thiz->drcParaMid.compressorRatio = mid.compressor_ratio;
    thiz->drcParaMid.compressorKneeWidth = mid.compressor_kneewidth;
    thiz->drcParaMid.expanderThreshold = mid.expander_threshhold;
    thiz->drcParaMid.expanderRatio = mid.expander_ratio;
    thiz->drcParaMid.expanderKneeWidth = mid.expander_kneewidth;
    thiz->drcParaMid.alphaA = mid.alpha_a;
    thiz->drcParaMid.betaA = mid.beta_a;
    thiz->drcParaMid.alphaR = mid.alpha_r;
    thiz->drcParaMid.betaR = mid.beta_r;
    thiz->drcParaMid.makeupGain = mid.gain;

    thiz->drcParaHi.enable = high.enable;
    thiz->drcParaHi.compressorThreshold = high.compressor_threshhold;
    thiz->drcParaHi.compressorRatio = high.compressor_ratio;
    thiz->drcParaHi.compressorKneeWidth = high.compressor_kneewidth;
    thiz->drcParaHi.expanderThreshold = high.expander_threshhold;
    thiz->drcParaHi.expanderRatio = high.expander_ratio;
    thiz->drcParaHi.expanderKneeWidth = high.expander_kneewidth;
    thiz->drcParaHi.alphaA = high.alpha_a;
    thiz->drcParaHi.betaA = high.beta_a;
    thiz->drcParaHi.alphaR = high.alpha_r;
    thiz->drcParaHi.betaR = high.beta_r;
    thiz->drcParaHi.makeupGain = high.gain;
    for (int i = 0; i < 32 * 5; i ++)
    {
        thiz->eq_coef_f[i] = 0.0f;
    }

    eq_coef2float(thiz->eq_coef_f, eq_coef, eq_num_stage);

#if BSP_USING_CMSIS_DSP
    thiz->dsp_coeffs_crossover_lpf[5] = thiz->dsp_coeffs_crossover_lpf[0] = crossover_lpf_fb[0];
    thiz->dsp_coeffs_crossover_lpf[6] = thiz->dsp_coeffs_crossover_lpf[1] = crossover_lpf_fb[1];
    thiz->dsp_coeffs_crossover_lpf[7] = thiz->dsp_coeffs_crossover_lpf[2] = crossover_lpf_fb[2];
    thiz->dsp_coeffs_crossover_lpf[8] = thiz->dsp_coeffs_crossover_lpf[3] = 0.0f - crossover_lpf_fa[0];
    thiz->dsp_coeffs_crossover_lpf[9] = thiz->dsp_coeffs_crossover_lpf[4] = 0.0f - crossover_lpf_fa[1];
    arm_biquad_cascade_df1_init_f32(&thiz->dsp_inst_crossover_lpf_left,
                                    2,
                                    thiz->dsp_coeffs_crossover_lpf,
                                    thiz->crossover_lpf_state_left);

    arm_biquad_cascade_df1_init_f32(&thiz->dsp_inst_crossover_lpf_right,
                                    2,
                                    thiz->dsp_coeffs_crossover_lpf,
                                    thiz->crossover_lpf_state_right);

    thiz->dsp_coeffs_crossover_hpf[5] = thiz->dsp_coeffs_crossover_hpf[0] = crossover_hpf_fb[0];
    thiz->dsp_coeffs_crossover_hpf[6] = thiz->dsp_coeffs_crossover_hpf[1] = crossover_hpf_fb[1];
    thiz->dsp_coeffs_crossover_hpf[7] = thiz->dsp_coeffs_crossover_hpf[2] = crossover_hpf_fb[2];
    thiz->dsp_coeffs_crossover_hpf[8] = thiz->dsp_coeffs_crossover_hpf[3] = 0.0f - crossover_hpf_fa[0];
    thiz->dsp_coeffs_crossover_hpf[9] = thiz->dsp_coeffs_crossover_hpf[4] = 0.0f - crossover_hpf_fa[1];
    arm_biquad_cascade_df1_init_f32(&thiz->dsp_inst_crossover_hpf_left,
                                    2,
                                    thiz->dsp_coeffs_crossover_hpf,
                                    thiz->crossover_hpf_state_left);

    arm_biquad_cascade_df1_init_f32(&thiz->dsp_inst_crossover_hpf_right,
                                    2,
                                    thiz->dsp_coeffs_crossover_hpf,
                                    thiz->crossover_hpf_state_right);

    thiz->dsp_coeffs_crossover_lpf1[0] = crossover_lpf1_fb[0];
    thiz->dsp_coeffs_crossover_lpf1[1] = crossover_lpf1_fb[1];
    thiz->dsp_coeffs_crossover_lpf1[2] = crossover_lpf1_fb[2];
    thiz->dsp_coeffs_crossover_lpf1[3] = 0.0f - crossover_lpf1_fa[0];
    thiz->dsp_coeffs_crossover_lpf1[4] = 0.0f - crossover_lpf1_fa[1];
    thiz->dsp_coeffs_crossover_lpf1[5] = crossover_lpf1_fb[3];
    thiz->dsp_coeffs_crossover_lpf1[6] = crossover_lpf1_fb[4];
    thiz->dsp_coeffs_crossover_lpf1[7] = crossover_lpf1_fb[5];
    thiz->dsp_coeffs_crossover_lpf1[8] = 0.0f - crossover_lpf1_fa[2];
    thiz->dsp_coeffs_crossover_lpf1[9] = 0.0f - crossover_lpf1_fa[3];
    thiz->dsp_coeffs_crossover_lpf1[10] = crossover_lpf1_fb[0];
    thiz->dsp_coeffs_crossover_lpf1[11] = crossover_lpf1_fb[1];
    thiz->dsp_coeffs_crossover_lpf1[12] = crossover_lpf1_fb[2];
    thiz->dsp_coeffs_crossover_lpf1[13] = 0.0f - crossover_lpf1_fa[0];
    thiz->dsp_coeffs_crossover_lpf1[14] = 0.0f - crossover_lpf1_fa[1];
    thiz->dsp_coeffs_crossover_lpf1[15] = crossover_lpf1_fb[3];
    thiz->dsp_coeffs_crossover_lpf1[16] = crossover_lpf1_fb[4];
    thiz->dsp_coeffs_crossover_lpf1[17] = crossover_lpf1_fb[5];
    thiz->dsp_coeffs_crossover_lpf1[18] = 0.0f - crossover_lpf1_fa[2];
    thiz->dsp_coeffs_crossover_lpf1[19] = 0.0f - crossover_lpf1_fa[3];


    arm_biquad_cascade_df1_init_f32(&thiz->dsp_inst_crossover_lpf1_left,
                                    4,
                                    thiz->dsp_coeffs_crossover_lpf1,
                                    thiz->crossover_lpf1_state_left);

    arm_biquad_cascade_df1_init_f32(&thiz->dsp_inst_crossover_lpf1_right,
                                    4,
                                    thiz->dsp_coeffs_crossover_lpf1,
                                    thiz->crossover_lpf1_state_right);

    thiz->dsp_coeffs_crossover_hpf1[0] = crossover_hpf1_fb[0];
    thiz->dsp_coeffs_crossover_hpf1[1] = crossover_hpf1_fb[1];
    thiz->dsp_coeffs_crossover_hpf1[2] = crossover_hpf1_fb[2];
    thiz->dsp_coeffs_crossover_hpf1[3] = 0.0f - crossover_hpf1_fa[0];
    thiz->dsp_coeffs_crossover_hpf1[4] = 0.0f - crossover_hpf1_fa[1];
    thiz->dsp_coeffs_crossover_hpf1[5] = crossover_hpf1_fb[3];
    thiz->dsp_coeffs_crossover_hpf1[6] = crossover_hpf1_fb[4];
    thiz->dsp_coeffs_crossover_hpf1[7] = crossover_hpf1_fb[5];
    thiz->dsp_coeffs_crossover_hpf1[8] = 0.0f - crossover_hpf1_fa[2];
    thiz->dsp_coeffs_crossover_hpf1[9] = 0.0f - crossover_hpf1_fa[3];
    thiz->dsp_coeffs_crossover_hpf1[10] = crossover_hpf1_fb[0];
    thiz->dsp_coeffs_crossover_hpf1[11] = crossover_hpf1_fb[1];
    thiz->dsp_coeffs_crossover_hpf1[12] = crossover_hpf1_fb[2];
    thiz->dsp_coeffs_crossover_hpf1[13] = 0.0f - crossover_hpf1_fa[0];
    thiz->dsp_coeffs_crossover_hpf1[14] = 0.0f - crossover_hpf1_fa[1];
    thiz->dsp_coeffs_crossover_hpf1[15] = crossover_hpf1_fb[3];
    thiz->dsp_coeffs_crossover_hpf1[16] = crossover_hpf1_fb[4];
    thiz->dsp_coeffs_crossover_hpf1[17] = crossover_hpf1_fb[5];
    thiz->dsp_coeffs_crossover_hpf1[18] = 0.0f - crossover_hpf1_fa[2];
    thiz->dsp_coeffs_crossover_hpf1[19] = 0.0f - crossover_hpf1_fa[3];

    arm_biquad_cascade_df1_init_f32(&thiz->dsp_inst_crossover_hpf1_left,
                                    4,
                                    thiz->dsp_coeffs_crossover_hpf1,
                                    thiz->crossover_hpf1_state_left);

    arm_biquad_cascade_df1_init_f32(&thiz->dsp_inst_crossover_hpf1_right,
                                    4,
                                    thiz->dsp_coeffs_crossover_hpf1,
                                    thiz->crossover_hpf1_state_right);

    thiz->dsp_coeffs_crossover_lpf2[0] = crossover_lpf2_fb[0];
    thiz->dsp_coeffs_crossover_lpf2[1] = crossover_lpf2_fb[1];
    thiz->dsp_coeffs_crossover_lpf2[2] = crossover_lpf2_fb[2];
    thiz->dsp_coeffs_crossover_lpf2[3] = 0.0f - crossover_lpf2_fa[0];
    thiz->dsp_coeffs_crossover_lpf2[4] = 0.0f - crossover_lpf2_fa[1];
    thiz->dsp_coeffs_crossover_lpf2[5] = crossover_lpf2_fb[3];
    thiz->dsp_coeffs_crossover_lpf2[6] = crossover_lpf2_fb[4];
    thiz->dsp_coeffs_crossover_lpf2[7] = crossover_lpf2_fb[5];
    thiz->dsp_coeffs_crossover_lpf2[8] = 0.0f - crossover_lpf2_fa[2];
    thiz->dsp_coeffs_crossover_lpf2[9] = 0.0f - crossover_lpf2_fa[3];
    thiz->dsp_coeffs_crossover_lpf2[10] = crossover_lpf2_fb[0];
    thiz->dsp_coeffs_crossover_lpf2[11] = crossover_lpf2_fb[1];
    thiz->dsp_coeffs_crossover_lpf2[12] = crossover_lpf2_fb[2];
    thiz->dsp_coeffs_crossover_lpf2[13] = 0.0f - crossover_lpf2_fa[0];
    thiz->dsp_coeffs_crossover_lpf2[14] = 0.0f - crossover_lpf2_fa[1];
    thiz->dsp_coeffs_crossover_lpf2[15] = crossover_lpf2_fb[3];
    thiz->dsp_coeffs_crossover_lpf2[16] = crossover_lpf2_fb[4];
    thiz->dsp_coeffs_crossover_lpf2[17] = crossover_lpf2_fb[5];
    thiz->dsp_coeffs_crossover_lpf2[18] = 0.0f - crossover_lpf2_fa[2];
    thiz->dsp_coeffs_crossover_lpf2[19] = 0.0f - crossover_lpf2_fa[3];
    arm_biquad_cascade_df1_init_f32(&thiz->dsp_inst_crossover_lpf2u_left,
                                    4,
                                    thiz->dsp_coeffs_crossover_lpf2,
                                    thiz->crossover_lpf2u_state_left);
    arm_biquad_cascade_df1_init_f32(&thiz->dsp_inst_crossover_lpf2u_right,
                                    4,
                                    thiz->dsp_coeffs_crossover_lpf2,
                                    thiz->crossover_lpf2u_state_right);
    arm_biquad_cascade_df1_init_f32(&thiz->dsp_inst_crossover_lpf2d_left,
                                    4,
                                    thiz->dsp_coeffs_crossover_lpf2,
                                    thiz->crossover_lpf2d_state_left);
    arm_biquad_cascade_df1_init_f32(&thiz->dsp_inst_crossover_lpf2d_right,
                                    4,
                                    thiz->dsp_coeffs_crossover_lpf2,
                                    thiz->crossover_lpf2d_state_right);

    thiz->dsp_coeffs_crossover_hpf2[0] = crossover_hpf2_fb[0];
    thiz->dsp_coeffs_crossover_hpf2[1] = crossover_hpf2_fb[1];
    thiz->dsp_coeffs_crossover_hpf2[2] = crossover_hpf2_fb[2];
    thiz->dsp_coeffs_crossover_hpf2[3] = 0.0f - crossover_hpf2_fa[0];
    thiz->dsp_coeffs_crossover_hpf2[4] = 0.0f - crossover_hpf2_fa[1];
    thiz->dsp_coeffs_crossover_hpf2[5] = crossover_hpf2_fb[3];
    thiz->dsp_coeffs_crossover_hpf2[6] = crossover_hpf2_fb[4];
    thiz->dsp_coeffs_crossover_hpf2[7] = crossover_hpf2_fb[5];
    thiz->dsp_coeffs_crossover_hpf2[8] = 0.0f - crossover_hpf2_fa[2];
    thiz->dsp_coeffs_crossover_hpf2[9] = 0.0f - crossover_hpf2_fa[3];
    thiz->dsp_coeffs_crossover_hpf2[10] = crossover_hpf2_fb[0];
    thiz->dsp_coeffs_crossover_hpf2[11] = crossover_hpf2_fb[1];
    thiz->dsp_coeffs_crossover_hpf2[12] = crossover_hpf2_fb[2];
    thiz->dsp_coeffs_crossover_hpf2[13] = 0.0f - crossover_hpf2_fa[0];
    thiz->dsp_coeffs_crossover_hpf2[14] = 0.0f - crossover_hpf2_fa[1];
    thiz->dsp_coeffs_crossover_hpf2[15] = crossover_hpf2_fb[3];
    thiz->dsp_coeffs_crossover_hpf2[16] = crossover_hpf2_fb[4];
    thiz->dsp_coeffs_crossover_hpf2[17] = crossover_hpf2_fb[5];
    thiz->dsp_coeffs_crossover_hpf2[18] = 0.0f - crossover_hpf2_fa[2];
    thiz->dsp_coeffs_crossover_hpf2[19] = 0.0f - crossover_hpf2_fa[3];

    arm_biquad_cascade_df1_init_f32(&thiz->dsp_inst_crossover_hpf2u_left,
                                    4,
                                    thiz->dsp_coeffs_crossover_hpf2,
                                    thiz->crossover_hpf2u_state_left);
    arm_biquad_cascade_df1_init_f32(&thiz->dsp_inst_crossover_hpf2u_right,
                                    4,
                                    thiz->dsp_coeffs_crossover_hpf2,
                                    thiz->crossover_hpf2u_state_right);
    arm_biquad_cascade_df1_init_f32(&thiz->dsp_inst_crossover_hpf2d_left,
                                    4,
                                    thiz->dsp_coeffs_crossover_hpf2,
                                    thiz->crossover_hpf2d_state_left);
    arm_biquad_cascade_df1_init_f32(&thiz->dsp_inst_crossover_hpf2d_right,
                                    4,
                                    thiz->dsp_coeffs_crossover_hpf2,
                                    thiz->crossover_hpf2d_state_right);

    thiz->dsp_coeffs_slope_lpf[0] = slope_lpf_fb[0];
    thiz->dsp_coeffs_slope_lpf[1] = slope_lpf_fb[1];
    thiz->dsp_coeffs_slope_lpf[2] = slope_lpf_fb[2];
    thiz->dsp_coeffs_slope_lpf[3] = 0.0f - slope_lpf_fa[0];
    thiz->dsp_coeffs_slope_lpf[4] = 0.0f - slope_lpf_fa[1];
    thiz->dsp_coeffs_slope_lpf[5] = slope_lpf_fb[3];
    thiz->dsp_coeffs_slope_lpf[6] = slope_lpf_fb[4];
    thiz->dsp_coeffs_slope_lpf[7] = slope_lpf_fb[5];
    thiz->dsp_coeffs_slope_lpf[8] = 0.0f - slope_lpf_fa[2];
    thiz->dsp_coeffs_slope_lpf[9] = 0.0f - slope_lpf_fa[3];
    thiz->dsp_coeffs_slope_lpf[10] = slope_lpf_fb[6];
    thiz->dsp_coeffs_slope_lpf[11] = slope_lpf_fb[7];
    thiz->dsp_coeffs_slope_lpf[12] = slope_lpf_fb[8];
    thiz->dsp_coeffs_slope_lpf[13] = 0.0f - slope_lpf_fa[4];
    thiz->dsp_coeffs_slope_lpf[14] = 0.0f - slope_lpf_fa[5];
    thiz->dsp_coeffs_slope_lpf[15] = slope_lpf_fb[9];
    thiz->dsp_coeffs_slope_lpf[16] = slope_lpf_fb[10];
    thiz->dsp_coeffs_slope_lpf[17] = slope_lpf_fb[11];
    thiz->dsp_coeffs_slope_lpf[18] = 0.0f - slope_lpf_fa[6];
    thiz->dsp_coeffs_slope_lpf[19] = 0.0f - slope_lpf_fa[7];

    arm_biquad_cascade_df1_init_f32(&thiz->dsp_inst_slope_lpf,
                                    4,
                                    thiz->dsp_coeffs_slope_lpf,
                                    thiz->slope_lpf_state);

    thiz->dsp_coeffs_slope_hpf[0] = slope_hpf_fb[0];
    thiz->dsp_coeffs_slope_hpf[1] = slope_hpf_fb[1];
    thiz->dsp_coeffs_slope_hpf[2] = slope_hpf_fb[2];
    thiz->dsp_coeffs_slope_hpf[3] = 0.0f - slope_hpf_fa[0];
    thiz->dsp_coeffs_slope_hpf[4] = 0.0f - slope_hpf_fa[1];
    thiz->dsp_coeffs_slope_hpf[5] = slope_hpf_fb[3];
    thiz->dsp_coeffs_slope_hpf[6] = slope_hpf_fb[4];
    thiz->dsp_coeffs_slope_hpf[7] = slope_hpf_fb[5];
    thiz->dsp_coeffs_slope_hpf[8] = 0.0f - slope_hpf_fa[2];
    thiz->dsp_coeffs_slope_hpf[9] = 0.0f - slope_hpf_fa[3];
    arm_biquad_cascade_df1_init_f32(&thiz->dsp_inst_slope_hpf,
                                    2,
                                    thiz->dsp_coeffs_slope_hpf,
                                    thiz->slope_hpf_state);
#endif


}

void *vbe_drc_open(int samplerate, uint8_t channels, uint8_t bit_width)
{
    vbe_drc_t *thiz = (vbe_drc_t *)vbe_malloc(sizeof(vbe_drc_t));
    if (!thiz)
    {
        return NULL;
    }

    memset(thiz, 0, sizeof(vbe_drc_t));

    thiz->bit_width = bit_width;

    RT_ASSERT(bit_width == 16);

    thiz->drc_gs_low_left = 0.0f;
    thiz->drc_gs_mid_left = 0.0f;
    thiz->drc_gs_hi_left = 0.0f;
    thiz->drc_gs_low_right = 0.0f;
    thiz->drc_gs_mid_right = 0.0f;
    thiz->drc_gs_hi_right = 0.0f;

    for (int i = 0; i < 8; i++)
    {
        thiz->crossover_lpf_state_left[i] = 0.0f;
        thiz->crossover_lpf_state_right[i] = 0.0f;
        thiz->crossover_hpf_state_left[i] = 0.0f;
        thiz->crossover_hpf_state_right[i] = 0.0f;
        thiz->slope_lpf_state[i] = 0.0f;
        thiz->slope_lpf_state[i + 8] = 0.0f;
        thiz->slope_hpf_state[i] = 0.0f;
    }

    for (int i = 0; i < 16; i++)
    {
        thiz->crossover_lpf1_state_left[i] = 0.0f;
        thiz->crossover_hpf1_state_left[i] = 0.0f;
        thiz->crossover_lpf2u_state_left[i] = 0.0f;
        thiz->crossover_hpf2u_state_left[i] = 0.0f;
        thiz->crossover_lpf2d_state_left[i] = 0.0f;
        thiz->crossover_hpf2d_state_left[i] = 0.0f;

        thiz->crossover_lpf1_state_right[i] = 0.0f;
        thiz->crossover_hpf1_state_right[i] = 0.0f;
        thiz->crossover_lpf2u_state_right[i] = 0.0f;
        thiz->crossover_hpf2u_state_right[i] = 0.0f;
        thiz->crossover_lpf2d_state_right[i] = 0.0f;
        thiz->crossover_hpf2d_state_right[i] = 0.0f;
    }
    for (int i = 0; i < 64; i++)
    {
        thiz->eq_state_left[i] = 0.0f;
        thiz->eq_state_right[i] = 0.0f;
    }
    thiz->channels              = channels;
    RT_ASSERT(thiz->channels == 1 || thiz->channels == 2);

    load_config(thiz);

    return (void *)thiz;
}

void vbe_drc_close(void *vbe)
{
    vbe_drc_t *thiz = (vbe_drc_t *)vbe;
    if (thiz)
    {
        vbe_free(thiz);
    }
}

static uint32_t proces_one_frame(vbe_drc_t *thiz, int16_t *out)
{
    // 1. vbe
    if (vbe_enable == 0)
    {
        for (int j = 0; j < VBE_ONE_FRAME_SAMPLES; j++)
        {
            thiz->x_hp_left[j] = thiz->audio_data_left[j];
            thiz->x_hp_right[j] = thiz->audio_data_right[j];
        }
        //skip vbe
        goto process_eq;
    }
    //1.1  do vbe
#if BSP_USING_CMSIS_DSP
    arm_biquad_cascade_df1_f32(&thiz->dsp_inst_crossover_lpf_left,
                               thiz->audio_data_left,
                               thiz->x_lp_left,
                               VBE_ONE_FRAME_SAMPLES);

    arm_biquad_cascade_df1_f32(&thiz->dsp_inst_crossover_lpf_right,
                               thiz->audio_data_right,
                               thiz->x_lp_right,
                               VBE_ONE_FRAME_SAMPLES);

    arm_biquad_cascade_df1_f32(&thiz->dsp_inst_crossover_hpf_left,
                               thiz->audio_data_left,
                               thiz->x_hp_left,
                               VBE_ONE_FRAME_SAMPLES);

    arm_biquad_cascade_df1_f32(&thiz->dsp_inst_crossover_hpf_right,
                               thiz->audio_data_right,
                               thiz->x_hp_right,
                               VBE_ONE_FRAME_SAMPLES);
#else
    crossover_lpf_left(thiz->x_lp_left,
                       thiz->audio_data_left,
                       VBE_ONE_FRAME_SAMPLES,
                       thiz->crossover_lpf_state_left,
                       crossover_lpf_fb,
                       crossover_lpf_fa);

    crossover_lpf_right(thiz->x_lp_right,
                        thiz->audio_data_right,
                        VBE_ONE_FRAME_SAMPLES,
                        thiz->crossover_lpf_state_right,
                        crossover_lpf_fb,
                        crossover_lpf_fa);

    crossover_hpf_left(thiz->x_hp_left,
                       thiz->audio_data_left,
                       VBE_ONE_FRAME_SAMPLES,
                       thiz->crossover_hpf_state_left,
                       crossover_hpf_fb,
                       crossover_hpf_fa);

    crossover_hpf_right(thiz->x_hp_right,
                        thiz->audio_data_right,
                        VBE_ONE_FRAME_SAMPLES,
                        thiz->crossover_hpf_state_right,
                        crossover_hpf_fb,
                        crossover_hpf_fa);

#endif
    for (int j = 0; j < VBE_ONE_FRAME_SAMPLES; j++)
    {
        thiz->x_mono[j] = 0.5f * (thiz->x_lp_left[j] + thiz->x_lp_right[j]);
    }

    vbe_func(vbe_gain, thiz->x_mono, VBE_ONE_FRAME_SAMPLES);

#if BSP_USING_CMSIS_DSP
    arm_biquad_cascade_df1_f32(&thiz->dsp_inst_slope_lpf,
                               thiz->x_mono,
                               thiz->x_mono_out,
                               VBE_ONE_FRAME_SAMPLES);

    arm_biquad_cascade_df1_f32(&thiz->dsp_inst_slope_hpf,
                               thiz->x_mono_out,
                               thiz->x_mono,
                               VBE_ONE_FRAME_SAMPLES);
#else
    slope_lpf(thiz->x_mono,
              VBE_ONE_FRAME_SAMPLES,
              thiz->slope_lpf_state,
              slope_lpf_fb,
              slope_lpf_fa);

    slope_hpf(thiz->x_mono,
              VBE_ONE_FRAME_SAMPLES,
              thiz->slope_hpf_state,
              slope_hpf_fb,
              slope_hpf_fa);
#endif

    for (int j = 0; j < VBE_ONE_FRAME_SAMPLES; j++)
    {
        thiz->x_hp_left[j] += thiz->x_mono[j];
        thiz->x_hp_right[j] += thiz->x_mono[j];
    }

process_eq:

    // soft eq filter
    if (eq_enable == 1)
    {
        eq_filter_left(thiz->eq_out_left, thiz->x_hp_left, VBE_ONE_FRAME_SAMPLES, thiz->eq_state_left, thiz->eq_coef_f, eq_num_stage);
        eq_filter_right(thiz->eq_out_right, thiz->x_hp_right, VBE_ONE_FRAME_SAMPLES, thiz->eq_state_right, thiz->eq_coef_f, eq_num_stage);
    }
    else
    {
        for (int j = 0; j < VBE_ONE_FRAME_SAMPLES; j++)
        {
            thiz->eq_out_left[j] = thiz->x_hp_left[j];
            thiz->eq_out_right[j] = thiz->x_hp_right[j];
        }
    }

    //drc three-band crossover filter, left channel
    if (thiz->drc_enable == 1)
    {
#if 0 //BSP_USING_CMSIS_DSP
        arm_biquad_cascade_df1_f32(&thiz->dsp_inst_crossover_lpf1_left,
                                   thiz->audio_data_left,
                                   thiz->crossover_lpf1_out,
                                   VBE_ONE_FRAME_SAMPLES);

        arm_biquad_cascade_df1_f32(&thiz->dsp_inst_crossover_hpf1_left,
                                   thiz->audio_data_left,
                                   thiz->crossover_hpf1_out,
                                   VBE_ONE_FRAME_SAMPLES);

        arm_biquad_cascade_df1_f32(&thiz->dsp_inst_crossover_lpf2u_left,
                                   thiz->crossover_lpf1_out,
                                   thiz->crossover_low_left,
                                   VBE_ONE_FRAME_SAMPLES);

        arm_biquad_cascade_df1_f32(&thiz->dsp_inst_crossover_hpf2u_left,
                                   thiz->crossover_lpf1_out,
                                   thiz->crossover_mid_left,
                                   VBE_ONE_FRAME_SAMPLES);

        arm_biquad_cascade_df1_f32(&thiz->dsp_inst_crossover_lpf2d_left,
                                   thiz->crossover_hpf1_out,
                                   thiz->crossover_lpf2d_out,
                                   VBE_ONE_FRAME_SAMPLES);

        arm_biquad_cascade_df1_f32(&thiz->dsp_inst_crossover_hpf2d_left,
                                   thiz->crossover_hpf1_out,
                                   thiz->crossover_hpf2d_out,
                                   VBE_ONE_FRAME_SAMPLES);

#else
        crossover_lpf1_left(thiz->crossover_lpf1_out,
                            thiz->audio_data_left,
                            VBE_ONE_FRAME_SAMPLES,
                            thiz->crossover_lpf1_state_left,
                            crossover_lpf1_fb,
                            crossover_lpf1_fa);

        crossover_hpf1_left(thiz->crossover_hpf1_out,
                            thiz->audio_data_left,
                            VBE_ONE_FRAME_SAMPLES,
                            thiz->crossover_hpf1_state_left,
                            crossover_hpf1_fb,
                            crossover_hpf1_fa);

        crossover_lpf2u_left(thiz->crossover_low_left,
                             thiz->crossover_lpf1_out,
                             VBE_ONE_FRAME_SAMPLES,
                             thiz->crossover_lpf2u_state_left,
                             crossover_lpf2_fb,
                             crossover_lpf2_fa);

        crossover_hpf2u_left(thiz->crossover_mid_left,
                             thiz->crossover_lpf1_out,
                             VBE_ONE_FRAME_SAMPLES,
                             thiz->crossover_hpf2u_state_left,
                             crossover_hpf2_fb,
                             crossover_hpf2_fa);

        crossover_lpf2d_left(thiz->crossover_lpf2d_out,
                             thiz->crossover_hpf1_out,
                             VBE_ONE_FRAME_SAMPLES,
                             thiz->crossover_lpf2d_state_left,
                             crossover_lpf2_fb,
                             crossover_lpf2_fa);

        crossover_hpf2d_left(thiz->crossover_hpf2d_out,
                             thiz->crossover_hpf1_out,
                             VBE_ONE_FRAME_SAMPLES,
                             thiz->crossover_hpf2d_state_left,
                             crossover_hpf2_fb,
                             crossover_hpf2_fa);
#endif

        for (int j = 0; j < VBE_ONE_FRAME_SAMPLES; j++)
        {
            thiz->crossover_hi_left[j] = thiz->crossover_lpf2d_out[j] + thiz->crossover_hpf2d_out[j];
        }

#if 0 //BSP_USING_CMSIS_DSP
        arm_biquad_cascade_df1_f32(&thiz->dsp_inst_crossover_lpf1_right,
                                   thiz->audio_data_right,
                                   thiz->crossover_lpf1_out,
                                   VBE_ONE_FRAME_SAMPLES);

        arm_biquad_cascade_df1_f32(&thiz->dsp_inst_crossover_hpf1_right,
                                   thiz->audio_data_right,
                                   thiz->crossover_hpf1_out,
                                   VBE_ONE_FRAME_SAMPLES);
        arm_biquad_cascade_df1_f32(&thiz->dsp_inst_crossover_lpf2u_right,
                                   thiz->crossover_lpf1_out,
                                   thiz->crossover_low_right,
                                   VBE_ONE_FRAME_SAMPLES);

        arm_biquad_cascade_df1_f32(&thiz->dsp_inst_crossover_hpf2u_right,
                                   thiz->crossover_lpf1_out,
                                   thiz->crossover_mid_right,
                                   VBE_ONE_FRAME_SAMPLES);

        arm_biquad_cascade_df1_f32(&thiz->dsp_inst_crossover_lpf2d_right,
                                   thiz->crossover_hpf1_out,
                                   thiz->crossover_lpf2d_out,
                                   VBE_ONE_FRAME_SAMPLES);

        arm_biquad_cascade_df1_f32(&thiz->dsp_inst_crossover_hpf2d_right,
                                   thiz->crossover_hpf1_out,
                                   thiz->crossover_hpf2d_out,
                                   VBE_ONE_FRAME_SAMPLES);
#else
        //drc three-band crossover filter, right channel
        crossover_lpf1_right(thiz->crossover_lpf1_out,
                             thiz->audio_data_right,
                             VBE_ONE_FRAME_SAMPLES,
                             thiz->crossover_lpf1_state_right,
                             crossover_lpf1_fb,
                             crossover_lpf1_fa);

        crossover_hpf1_right(thiz->crossover_hpf1_out,
                             thiz->audio_data_right,
                             VBE_ONE_FRAME_SAMPLES,
                             thiz->crossover_hpf1_state_right,
                             crossover_hpf1_fb,
                             crossover_hpf1_fa);

        crossover_lpf2u_right(thiz->crossover_low_right,
                              thiz->crossover_lpf1_out,
                              VBE_ONE_FRAME_SAMPLES,
                              thiz->crossover_lpf2u_state_right,
                              crossover_lpf2_fb,
                              crossover_lpf2_fa);

        crossover_hpf2u_right(thiz->crossover_mid_right,
                              thiz->crossover_lpf1_out,
                              VBE_ONE_FRAME_SAMPLES,
                              thiz->crossover_hpf2u_state_right,
                              crossover_hpf2_fb,
                              crossover_hpf2_fa);

        crossover_lpf2d_right(thiz->crossover_lpf2d_out,
                              thiz->crossover_hpf1_out,
                              VBE_ONE_FRAME_SAMPLES,
                              thiz->crossover_lpf2d_state_right,
                              crossover_lpf2_fb,
                              crossover_lpf2_fa);

        crossover_hpf2d_right(thiz->crossover_hpf2d_out,
                              thiz->crossover_hpf1_out,
                              VBE_ONE_FRAME_SAMPLES,
                              thiz->crossover_hpf2d_state_right,
                              crossover_hpf2_fb,
                              crossover_hpf2_fa);
#endif

        for (int j = 0; j < VBE_ONE_FRAME_SAMPLES; j++)
        {
            thiz->crossover_hi_right[j] = thiz->crossover_lpf2d_out[j] + thiz->crossover_hpf2d_out[j];
        }

        // three-band drc
        if (low.enable == 1)
        {
            drc_low_left(thiz->drc_low_left_out, thiz->crossover_low_left, &thiz->drcParaLow, &thiz->drc_gs_low_left, VBE_ONE_FRAME_SAMPLES);
            drc_low_right(thiz->drc_low_right_out, thiz->crossover_low_right, &thiz->drcParaLow, &thiz->drc_gs_low_right, VBE_ONE_FRAME_SAMPLES);
        }
        else
        {
            for (int j = 0; j < VBE_ONE_FRAME_SAMPLES; j++)
            {
                thiz->drc_low_left_out[j] = thiz->crossover_low_left[j];
                thiz->drc_low_right_out[j] = thiz->crossover_low_right[j];
            }
        }
        if (mid.enable == 1)
        {
            drc_mid_left(thiz->drc_mid_left_out, thiz->crossover_mid_left, &thiz->drcParaMid, &thiz->drc_gs_mid_left, VBE_ONE_FRAME_SAMPLES);
            drc_mid_right(thiz->drc_mid_right_out, thiz->crossover_mid_right, &thiz->drcParaMid, &thiz->drc_gs_mid_right, VBE_ONE_FRAME_SAMPLES);
        }
        else
        {
            for (int j = 0; j < VBE_ONE_FRAME_SAMPLES; j++)
            {
                thiz->drc_mid_left_out[j] = thiz->crossover_mid_left[j];
                thiz->drc_mid_right_out[j] = thiz->crossover_mid_right[j];
            }
        }

        if (high.enable == 1)
        {
            drc_hi_left(thiz->drc_hi_left_out, thiz->crossover_hi_left, &thiz->drcParaHi, &thiz->drc_gs_hi_left, VBE_ONE_FRAME_SAMPLES);
            drc_hi_right(thiz->drc_hi_right_out, thiz->crossover_hi_right, &thiz->drcParaHi, &thiz->drc_gs_hi_right, VBE_ONE_FRAME_SAMPLES);
        }
        else
        {
            for (int j = 0; j < VBE_ONE_FRAME_SAMPLES; j++)
            {
                thiz->drc_hi_left_out[j] = thiz->crossover_hi_left[j];
                thiz->drc_hi_right_out[j] = thiz->crossover_hi_right[j];
            }
        }
        // drc out
        for (int j = 0; j < VBE_ONE_FRAME_SAMPLES; j++)
        {
            thiz->drc_left_out[j] = thiz->drc_low_left_out[j] + thiz->drc_mid_left_out[j] + thiz->drc_hi_left_out[j];
            thiz->drc_right_out[j] = thiz->drc_low_right_out[j] + thiz->drc_mid_right_out[j] + thiz->drc_hi_right_out[j];

            thiz->drc_left_out[j] = thiz->drc_left_out[j];   //(float)(floor(thiz->drc_left_out[j] + 0.5f));
            thiz->drc_right_out[j] = thiz->drc_right_out[j]; //(float)(floor(thiz->drc_right_out[j] + 0.5f));
        }
    }
    else
    {
        for (int j = 0; j < VBE_ONE_FRAME_SAMPLES; j++)
        {
            thiz->drc_left_out[j] = thiz->eq_out_left[j];
            thiz->drc_right_out[j] = thiz->eq_out_right[j];
        }
    }
    for (int j = 0; j < VBE_ONE_FRAME_SAMPLES; j++)
    {
        if (thiz->drc_left_out[j] > ((1 << (thiz->bit_width - 1)) - 1))
        {
            thiz->drc_left_out[j] = (1 << (thiz->bit_width - 1)) - 1;
        }
        else if (thiz->drc_left_out[j] < -((1 << (thiz->bit_width - 1)) - 1))
        {
            thiz->drc_left_out[j] = -((1 << (thiz->bit_width - 1)) - 1);
        }

        if (thiz->drc_right_out[j] > ((1 << (thiz->bit_width - 1)) - 1))
        {
            thiz->drc_right_out[j] = (1 << (thiz->bit_width - 1)) - 1;
        }
        else if (thiz->drc_right_out[j] < -((1 << (thiz->bit_width - 1)) - 1))
        {
            thiz->drc_right_out[j] = -((1 << (thiz->bit_width - 1)) - 1);
        }
    }


    uint32_t all_channel_samples = VBE_ONE_FRAME_SAMPLES;
    if (thiz->channels == 2)
    {
        all_channel_samples <<= 1;
        for (int j = 0; j < VBE_ONE_FRAME_SAMPLES; j++)
        {
            *out++ = (int16_t)(thiz->drc_left_out[j]);
            *out++ = (int16_t)(thiz->drc_right_out[j]);

        }
    }
    else
    {
        for (int j = 0; j < VBE_ONE_FRAME_SAMPLES; j++)
        {
            *out++ = (int16_t)(thiz->drc_left_out[j]);
        }
    }
    return all_channel_samples;
}

int vbe_drc_process(void *vbe, int16_t *data, uint16_t samples, int16_t *out, uint32_t out_size)
{
    uint32_t copy_samples = 0, vbe_samples = 0, out_samples = 0;
    vbe_drc_t *thiz = (vbe_drc_t *)vbe;

    if (!thiz || !data || !samples)
    {
        return 0;
    }

    if (thiz->should_update_config)
    {
        load_config(thiz);
    }

    uint32_t max_out = samples * 2 + VBE_ONE_FRAME_SAMPLES * 4;
    if (max_out > out_size)
    {
        rt_kprintf("m=%d o=%d\n", max_out, out_size);
        RT_ASSERT(0);
        return 0;
    }

    if (thiz->channels == 2)
    {
        RT_ASSERT((samples & 1) == 0);
        samples >>= 1;
    }

    copy_samples = VBE_ONE_FRAME_SAMPLES - thiz->remain_samples;
    if (samples < copy_samples)
    {
        copy_samples = samples;
    }

    uint16_t start = thiz->remain_samples;
    uint16_t end   = start + copy_samples;

    if (thiz->channels == 2)
    {
        while (start < end)
        {
            thiz->audio_data_left[start] = (float)(*data++);
            thiz->audio_data_right[start++] = (float)(*data++);
        }
    }
    else
    {
        while (start < end)
        {
            thiz->audio_data_left[start] = (float)(*data);
            thiz->audio_data_right[start++] = (float)(*data++);
        }
    }
    thiz->remain_samples += copy_samples;
    if (thiz->remain_samples < VBE_ONE_FRAME_SAMPLES)
    {
        return 0;
    }

    RT_ASSERT(thiz->remain_samples == VBE_ONE_FRAME_SAMPLES);


    vbe_samples = proces_one_frame(thiz, out);
    out += vbe_samples;
    out_samples += vbe_samples;

    thiz->remain_samples = 0;
    samples -= copy_samples;

    while (samples > 0)
    {
        start = 0;
        end   = VBE_ONE_FRAME_SAMPLES;
        if (samples < VBE_ONE_FRAME_SAMPLES)
        {
            end = samples;
        }
        if (thiz->channels == 2)
        {
            while (start < end)
            {
                thiz->audio_data_left[start] = (float)(*data++);
                thiz->audio_data_right[start++] = (float)(*data++);
            }
        }
        else
        {
            while (start < end)
            {
                thiz->audio_data_left[start] = (float)(*data);
                thiz->audio_data_right[start++] = (float)(*data++);
            }
        }
        if (samples < VBE_ONE_FRAME_SAMPLES)
        {
            thiz->remain_samples = samples;
            break;
        }
        samples -= end;
        if (end == VBE_ONE_FRAME_SAMPLES)
        {
            vbe_samples = proces_one_frame(thiz, out);
            out += vbe_samples;
            out_samples += vbe_samples;
        }
        else
        {
            RT_ASSERT(samples == 0);
        }
    }

    return (int)out_samples * 2;
}

#ifdef RT_USING_FINSH

static const char str_eq[] = "eq";
static const char str_enable[] = "enable";
static const char str_state[] = "state";
static const char str_vbe[] = "vbe";
static const char str_drc[] = "drc";
static const char str_gain[] = "gain";
static inline int set_eq(int argc, char *argv[])
{
    char str[3] = {0};
    int index = strtoul(argv[4], 0, 10);
    if (argc < 6)
    {
        LOG_I("input para num %d < 4", argc);
        return -1;
    }

    if (strlen(argv[5]) != 40)
    {
        LOG_I("argv[5] len error");
        return -1;
    }

    if ((index >= 32) || (index < 0))
    {
        LOG_I("argv[4] %d not in 0-9", argv[2]);
        return -1;
    }

    uint8_t *pData = (uint8_t *)&eq_coef[index * 5];
    for (int m = 0; m < 20; m++)
    {
        str[0] = argv[5][m * 2];
        str[1] = argv[5][m * 2 + 1];

        pData[m] = strtoul(str, 0, 16);
    }
    LOG_I("drc_eq_w_eq %d success", index);
    return 0;
}

static inline int get_eq(char *value)
{
    int index = strtoul(value, 0, 10);

    LOG_I("drc_eq_r_eq:index=%d,0x%08x,0x%08x,0x%08x,0x%08x,0x%08x",
          index,
          eq_coef[index * 5],
          eq_coef[index * 5 + 1],
          eq_coef[index * 5 + 2],
          eq_coef[index * 5 + 3],
          eq_coef[index * 5 + 4]);

    return 0;
}

static int vbe_cmd(int argc, char *argv[])
{
    int ret = 0;
    if (argv[2][0] == 'w'
            && !strcmp(str_enable, argv[3])
            && argc == 5)
    {
        vbe_enable = atoi(argv[4]);
        LOG_I("drc_vbe_w_enable success");
    }
    else if (argv[2][0] == 'r'
             && !strcmp(str_enable, argv[3]))
    {
        LOG_I("drc_vbe_r_enable=%d", vbe_enable);
    }

    else if (argv[2][0] == 'w'
             && !strcmp(str_gain, argv[3])
             && argc == 5)
    {
        vbe_gain = atof(argv[4]);
        LOG_I("drc_vbe_w_gain success");
    }
    else if (argv[2][0] == 'r'
             && !strcmp(str_gain, argv[3]))
    {
        LOG_I("drc_vbe_r_gain=%f", vbe_gain);
    }
    else if (argv[2][0] == 'w'
             && !strcmp("clpf_fb", argv[3])
             && argc == 7)
    {
        crossover_lpf_fb[0] = atof(argv[4]);
        crossover_lpf_fb[1] = atof(argv[5]);
        crossover_lpf_fb[2] = atof(argv[6]);
        LOG_I("drc_vbe_w_clpf_fb success");
    }
    else if (argv[2][0] == 'r'
             && !strcmp("clpf_fb", argv[3]))
    {
        LOG_I("drc_vbe_r_clpf_fb=%f,%f,%f", crossover_lpf_fb[0], crossover_lpf_fb[1], crossover_lpf_fb[2]);
    }
    else if (argv[2][0] == 'w'
             && !strcmp("chpf_fb", argv[3])
             && argc == 7)
    {
        crossover_hpf_fb[0] = atof(argv[4]);
        crossover_hpf_fb[1] = atof(argv[5]);
        crossover_hpf_fb[2] = atof(argv[6]);
        LOG_I("drc_vbe_w_hpf_fb success");
    }
    else if (argv[2][0] == 'r'
             && !strcmp("chpf_fb", argv[3]))
    {
        LOG_I("drc_vbe_r_hpf_fb=%f,%f,%f", crossover_hpf_fb[0], crossover_hpf_fb[1], crossover_hpf_fb[2]);
    }
    else if (argv[2][0] == 'w'
             && !strcmp("clpf_fa", argv[3])
             && argc == 6)
    {
        crossover_lpf_fa[0] = atof(argv[4]);
        crossover_lpf_fa[1] = atof(argv[5]);
        LOG_I("drc_vbe_w_clpf_fa success");
    }
    else if (argv[2][0] == 'r'
             && !strcmp("clpf_fa", argv[3]))
    {
        LOG_I("drc_vbe_r_clpf_fa=%f,%f", crossover_lpf_fa[0], crossover_lpf_fa[1]);
    }
    else if (argv[2][0] == 'w'
             && !strcmp("chpf_fa", argv[3])
             && argc == 6)
    {
        crossover_hpf_fa[0] = atof(argv[4]);
        crossover_hpf_fa[1] = atof(argv[5]);
        LOG_I("drc_vbe_w_chpf_fa success");
    }
    else if (argv[2][0] == 'r'
             && !strcmp("chpf_fa", argv[3]))
    {
        LOG_I("drc_vbe_r_chpf_fa=%f,%f", crossover_hpf_fa[0], crossover_hpf_fa[1]);
    }
    else if (argv[2][0] == 'w'
             && !strcmp("slpf_fb", argv[3])
             && argc == 9)
    {
        int index = atoi(argv[4]);
        if (index >= 0 && index < 3)
        {
            slope_lpf_fb[index * 4 + 0] = atof(argv[5]);
            slope_lpf_fb[index * 4 + 1] = atof(argv[6]);
            slope_lpf_fb[index * 4 + 2] = atof(argv[7]);
            slope_lpf_fb[index * 4 + 3] = atof(argv[8]);
            LOG_I("drc_vbe_w_slpf_fb %d success", index);
        }
        else
        {
            ret = -1;
        }
    }
    else if (argv[2][0] == 'r'
             && !strcmp("slpf_fb", argv[3])
             && argc == 5)
    {
        int index = atoi(argv[4]);
        if (index >= 0 && index < 3)
        {
            LOG_I("drc_vbe_r_slpf_fb=%d,%f,%f,%f,%f",
                  index,
                  slope_lpf_fb[index * 4 + 0],
                  slope_lpf_fb[index * 4 + 1],
                  slope_lpf_fb[index * 4 + 2],
                  slope_lpf_fb[index * 4 + 3]);
        }
        else
        {
            ret = -1;
        }
    }
    else if (argv[2][0] == 'w'
             && !strcmp("slpf_fa", argv[3])
             && argc == 9)
    {
        int index = atoi(argv[4]);
        if (index >= 0 && index < 2)
        {
            slope_lpf_fa[index * 4 + 0] = atof(argv[5]);
            slope_lpf_fa[index * 4 + 1] = atof(argv[6]);
            slope_lpf_fa[index * 4 + 2] = atof(argv[7]);
            slope_lpf_fa[index * 4 + 3] = atof(argv[8]);
            LOG_I("drc_vbe_w_slpf_fa %d success", index);
        }
        else
        {
            ret = -1;
        }
    }
    else if (argv[2][0] == 'r'
             && !strcmp("slpf_fa", argv[3])
             && argc == 5)
    {
        int index = atoi(argv[4]);
        if (index >= 0 && index < 2)
        {
            LOG_I("drc_vbe_r_slpf_fa=%d,%f,%f,%f,%f",
                  index,
                  slope_lpf_fa[index * 4 + 0],
                  slope_lpf_fa[index * 4 + 1],
                  slope_lpf_fa[index * 4 + 2],
                  slope_lpf_fa[index * 4 + 3]);
        }
        else
        {
            ret = -1;
        }
    }
    else if (argv[2][0] == 'w'
             && !strcmp("shpf_fb", argv[3])
             && argc == 8)
    {
        int index = atoi(argv[4]);
        if (index >= 0 && index < 2)
        {
            slope_hpf_fb[index * 3 + 0] = atof(argv[5]);
            slope_hpf_fb[index * 3 + 1] = atof(argv[6]);
            slope_hpf_fb[index * 3 + 2] = atof(argv[7]);
            LOG_I("drc_vbe_w_shpf_fb %d success", index);
        }
        else
        {
            ret = -1;
        }
    }
    else if (argv[2][0] == 'r'
             && !strcmp("shpf_fb", argv[3])
             && argc == 5)
    {
        int index = atoi(argv[4]);
        if (index >= 0 && index < 2)
        {
            LOG_I("drc_vbe_r_shpf_fb=%d,%f,%f,%f",
                  index,
                  slope_hpf_fb[index * 3 + 0],
                  slope_hpf_fb[index * 3 + 1],
                  slope_hpf_fb[index * 3 + 2]);
        }
        else
        {
            ret = -1;
        }
    }
    else if (argv[2][0] == 'w'
             && !strcmp("shpf_fa", argv[3])
             && argc == 9)
    {
        slope_hpf_fa[0] = atof(argv[5]);
        slope_hpf_fa[1] = atof(argv[6]);
        slope_hpf_fa[2] = atof(argv[7]);
        slope_hpf_fa[3] = atof(argv[8]);
        LOG_I("drc_vbe_w_shpf_fa success");
    }
    else if (argv[2][0] == 'r'
             && !strcmp("shpf_fa", argv[3]))
    {
        LOG_I("drc_vbe_r_shpf_fa=%f,%f,%f,%f",
              slope_hpf_fa[0],
              slope_hpf_fa[1],
              slope_hpf_fa[2],
              slope_hpf_fa[3]);
    }
    else
    {
        ret = -1;
    }
    return ret;
}

static int drc_cmd(int argc, char *argv[])
{
    int ret = 0;
    float *fb = NULL;
    float *fa = NULL;
    char  *str = NULL;
    drc_para_t *para = NULL;

    if (!strcmp("lpf1_fb", argv[3]))
    {
        fb = crossover_lpf1_fb;
        str = "lpf1_fb";
    }
    else if (!strcmp("hpf1_fb", argv[3]))
    {
        fb = crossover_hpf1_fb;
        str = "hpf1_fb";
    }
    else if (!strcmp("lpf2_fb", argv[3]))
    {
        fb = crossover_lpf2_fb;
        str = "lpf2_fb";
    }
    else if (!strcmp("hpf2_fb", argv[3]))
    {
        fb = crossover_hpf2_fb;
        str = "hpf2_fb";
    }
    else if (!strcmp("lpf1_fa", argv[3]))
    {
        fa = crossover_lpf1_fa;
        str = "lpf1_fa";
    }
    else if (!strcmp("hpf1_fa", argv[3]))
    {
        fa = crossover_hpf1_fa;
        str = "hpf1_fa";
    }
    else if (!strcmp("lpf2_fa", argv[3]))
    {
        fa = crossover_lpf2_fa;
        str = "lpf2_fa";
    }
    else if (!strcmp("hpf2_fa", argv[3]))
    {
        fa = crossover_hpf2_fa;
        str = "hpf2_fa";
    }
    else if (!strcmp("low", argv[3]))
    {
        para = &low;
        str = "low";
    }
    else if (!strcmp("mid", argv[3]))
    {
        para = &mid;
        str = "mid";
    }
    else if (!strcmp("hi", argv[3]))
    {
        para = &high;
        str = "hi";
    }
    else
    {
        ret = -1;
    }

    if (fb)
    {
        if (argv[2][0] == 'w' && argc == 8)
        {
            int index = atoi(argv[4]);
            if (index >= 0 && index < 2)
            {
                fb[index * 3 + 0] = atof(argv[5]);
                fb[index * 3 + 1] = atof(argv[6]);
                fb[index * 3 + 2] = atof(argv[7]);
                LOG_I("drc_drc_w_%s %d success", str, index);
            }
            else
            {
                ret = -1;
            }
        }
        else if (argv[2][0] == 'r' && argc == 5)
        {
            int index = atoi(argv[4]);
            if (index >= 0 && index < 2)
            {
                LOG_I("drc_drc_r_%s=%d,%f,%f,%f",
                      str,
                      index,
                      fb[index * 3 + 0],
                      fb[index * 3 + 1],
                      fb[index * 3 + 2]);
            }
            else
            {
                ret = -1;
            }
        }
    }
    else if (fa)
    {
        if (argv[2][0] == 'r')
        {
            LOG_I("drc_drc_r_%s=%f,%f,%f,%f",
                  str,
                  fa[0],
                  fa[1],
                  fa[2],
                  fa[3]);
        }
        else if (argv[2][0] == 'w' && argc == 8)
        {
            fa[0] = atof(argv[5]);
            fa[1] = atof(argv[6]);
            fa[2] = atof(argv[7]);
            fa[3] = atof(argv[8]);
            LOG_I("drc_drc_w_%s success", str);
        }

        else
        {
            ret = -1;
        }
    }
    else if (para)
    {
        if (argv[2][0] == 'w' && argc == 6)
        {
            if (!strcmp("enable", argv[4]))
            {
                para->enable = atoi(argv[5]);
                LOG_I("drc_drc_w_%s_enable success", str);
            }
            else if (!strcmp("gain", argv[4]))
            {
                para->gain = atof(argv[5]);
                LOG_I("drc_drc_w_%s_gain success", str);
            }
            else
            {
                float val = atof(argv[5]);
                if (!strcmp("com_thresh", argv[4]))
                {
                    para->compressor_threshhold = val;
                    LOG_I("drc_drc_w_%s_com_thresh success", str);
                }
                else if (!strcmp("com_ratio", argv[4]))
                {
                    para->compressor_ratio = val;
                    LOG_I("drc_drc_w_%s_com_ratio success", str);
                }
                else if (!strcmp("com_width", argv[4]))
                {
                    para->compressor_kneewidth = val;
                    LOG_I("drc_drc_w_%s_com_width success", str);
                }
                else if (!strcmp("exp_thresh", argv[4]))
                {
                    para->expander_threshhold = val;
                    LOG_I("drc_drc_w_%s_exp_thresh success", str);
                }
                else if (!strcmp("exp_thresh", argv[4]))
                {
                    para->expander_threshhold = val;
                    LOG_I("drc_drc_w_%s_exp_thresh success", str);
                }
                else if (!strcmp("exp_ratio", argv[4]))
                {
                    para->expander_ratio = val;
                    LOG_I("drc_drc_w_%s_exp_ratio success", str);
                }
                else if (!strcmp("exp_width", argv[4]))
                {
                    para->expander_kneewidth = val;
                    LOG_I("drc_drc_w_%s_exp_width success", str);
                }
                else if (!strcmp("alpha_a", argv[4]))
                {
                    para->alpha_a = val;
                    LOG_I("drc_drc_w_%s_alpha_a success", str);
                }
                else if (!strcmp("beta_a", argv[4]))
                {
                    para->beta_a = val;
                    LOG_I("drc_drc_w_%s_beta_a success", str);
                }
                else if (!strcmp("alpha_r", argv[4]))
                {
                    para->alpha_r = val;
                    LOG_I("drc_drc_w_%s_alpha_r success", str);
                }
                else if (!strcmp("beta_r", argv[4]))
                {
                    para->beta_r = val;
                    LOG_I("drc_drc_w_%s_beta_r success", str);
                }
                else
                {
                    ret = -1;
                }
            }
        }
        else if (argv[2][0] == 'r' && argc == 5)
        {
            if (!strcmp("enable", argv[4]))
            {
                LOG_I("drc_drc_r_%s_enable=%d", str, para->enable);
            }
            else if (!strcmp("gain", argv[4]))
            {
                LOG_I("drc_drc_r_%s_gain=%d", str, para->gain);
            }
            else if (!strcmp("com_thresh", argv[4]))
            {
                LOG_I("drc_drc_r_%s_com_thresh=%f", str, para->compressor_threshhold);
            }
            else if (!strcmp("com_ratio", argv[4]))
            {
                LOG_I("drc_drc_r_%s_com_ratio=%f", str, para->compressor_ratio);
            }
            else if (!strcmp("com_width", argv[4]))
            {
                LOG_I("drc_drc_r_%s_com_width=%f", str, para->compressor_kneewidth);
            }
            else if (!strcmp("exp_thresh", argv[4]))
            {
                LOG_I("drc_drc_r_%s_exp_thresh=%f", str, para->expander_threshhold);
            }
            else if (!strcmp("exp_thresh", argv[4]))
            {
                LOG_I("drc_drc_r_%s_exp_thresh=%f", str, para->expander_threshhold);
            }
            else if (!strcmp("exp_ratio", argv[4]))
            {
                LOG_I("drc_drc_r_%s_exp_ratio=%f", str, para->expander_ratio);
            }
            else if (!strcmp("exp_width", argv[4]))
            {
                LOG_I("drc_drc_r_%s_exp_width=%f", str, para->expander_kneewidth);
            }
            else if (!strcmp("alpha_a", argv[4]))
            {
                LOG_I("drc_drc_r_%s_alpha_a=%f", str, para->alpha_a);
            }
            else if (!strcmp("beta_a", argv[4]))
            {
                LOG_I("drc_drc_r_%s_beta_a=%f", str, para->beta_a);
            }
            else if (!strcmp("alpha_r", argv[4]))
            {
                LOG_I("drc_drc_r_%s_alpha_r=%f", str, para->alpha_r);
            }
            else if (!strcmp("beta_r", argv[4]))
            {
                LOG_I("drc_drc_r_%s_beta_r=%f", str, para->beta_r);
            }
            else
            {
                ret = -1;
            }
        }
        else
        {
            ret = -1;
        }
    }
    else
    {
        ret = -1;
    }

    return ret;
}
int drc(int argc, char *argv[])
{
    int ret = 0;
    if (argc < 4)
    {
        LOG_I("argc=%d", argc);
        ret = -1;
    }
    else if (!strcmp(str_eq, argv[1])
             && argv[2][0] == 'w'
             && !strcmp(str_enable, argv[3]))
    {
        eq_enable = atoi(argv[4]);
        LOG_I("drc_eq_w_enable success");
    }
    else if (!strcmp(str_eq, argv[1])
             && argv[2][0] == 'r'
             && !strcmp(str_enable, argv[3]))
    {
        LOG_I("drc_eq_r_enable=%d");
    }
    else if (!strcmp(str_eq, argv[1])
             && argv[2][0] == 'w'
             && !strcmp(str_eq, argv[3]))
    {
        ret = set_eq(argc, argv);
    }
    else if (!strcmp(str_eq, argv[1])
             && argv[2][0] == 'r'
             && !strcmp(str_eq, argv[3])
             && argc > 4)
    {
        ret = get_eq(argv[4]);
    }
    else if (!strcmp(str_vbe, argv[1]))
    {
        ret = vbe_cmd(argc, argv);
    }
    else if (!strcmp(str_drc, argv[1]))
    {
        ret = drc_cmd(argc, argv);
    }
    else
    {
        ret = -1;
    }

    if (ret == 0)
    {
        LOG_I("drc_tool_ok");
        return 0;
    }
    LOG_I("drc_tool_fail");
    return -1;
}
MSH_CMD_EXPORT(drc, drc tools);

#endif

