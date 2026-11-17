/*
 * rs_config.h -- RoadSense on STM32F411 (Black Pill): build switches and constants.
 * Values are the same as the ESP32 RoadSense v4 firmware (tuned on the hatchback,
 * frozen for sedan/SUV). Only the hardware-specific parts differ.
 */
#ifndef RS_CONFIG_H
#define RS_CONFIG_H

/* 1 = loose trigger + on-device random forest (keep/drop + class)
 * 0 = rules only (paper settings: K_LOCAL 6, heave-duration label)          */
#ifndef RS_USE_ML
#define RS_USE_ML 1
#endif

#define RS_ML_KEEP_PNONE      0.5f   /* keep an event if P(NONE) < this; 0.6 = more recall */

#define RS_ACC_LSB_PER_G      8192.0f  /* MPU6050 +-4 g */
#define RS_DT                 0.01f    /* 100 Hz */
#define RS_GRAV_CALIB_MS      1500u    /* phase A: learn gravity direction */
#define RS_STAT_CALIB_MS      6500u    /* phase B: learn noise statistics  */

#define RS_K_EVENT            5.0f
#if RS_USE_ML
#define RS_K_LOCAL            2.0f     /* loose trigger: the forest removes the extra false alarms */
#else
#define RS_K_LOCAL            6.0f
#endif
#define RS_K_HEAVE_TRIG       6.0f
#define RS_K_HEAVE            2.0f
#define RS_HEAVE_DUR_BRK_MS   450u
#define RS_ROUGH_BURST_HITS   3
#define RS_K_ROUGH_YELLOW     2.5f
#define RS_K_ROUGH_RED        4.0f

#define RS_PRE_MS             200u     /* analysis window before the trigger */
#define RS_POST_MS            1200u    /* analysis window after the trigger  */
#define RS_HIT_GAP_MS         60u

#define RS_HEAVE_LP_HZ        3.0f
#define RS_TC_BASELINE_S      30.0f
#define RS_TC_GRAVITY_S       5.0f

#define RS_RWIN               100      /* 1 s jerk ring for roughness */
#define RS_PRE_N              (RS_PRE_MS / 10u)

#endif
