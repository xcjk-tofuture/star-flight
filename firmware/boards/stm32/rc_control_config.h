#ifndef UAV_RC_CONTROL_CONFIG_H
#define UAV_RC_CONTROL_CONFIG_H
/* All thresholds use calibrated 1000..2000 values. CH7/8 are UI knobs. */
#define UAV_RC_MODE_LOW 1250u
#define UAV_RC_MODE_HIGH 1750u
#define UAV_RC_SAFETY_OFF 1250u
#define UAV_RC_SAFETY_ON 1750u
#define UAV_RC_ARM_THROTTLE 1050u
#define UAV_RC_UI_THROTTLE 1100u
#define UAV_RC_CENTER_LOW 1400u
#define UAV_RC_CENTER_HIGH 1600u
#define UAV_RC_ARM_YAW 1900u
#define UAV_RC_NEUTRAL_MS 500u
#define UAV_RC_ARM_HOLD_MS 1000u
#endif
