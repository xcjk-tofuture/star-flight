#ifndef UAV_RC_UI_H
#define UAV_RC_UI_H
#include <stdint.h>
typedef struct { uint16_t channels[8]; uint32_t sample_ms; uint8_t connected; } rc_ui_frame_t;
typedef struct {
    uint32_t now_ms;
    uint8_t state, remote_calibrating, remote_ranges_ready, saving, remote_capture_view;
    uint8_t screen, page, select_count, select_index;
    uint16_t view_count, view_index;
    uint16_t control_context;
} rc_ui_context_t;
enum { RC_UI_NEXT=1, RC_UI_PREVIOUS, RC_UI_ENTER, RC_UI_BACK, RC_UI_CAL,
       RC_UI_SELECT, RC_UI_VIEW, RC_UI_SAVE_DIALOG };
typedef struct { uint8_t kind; uint16_t value; } rc_ui_action_t;
typedef struct {
    uint32_t neutral_ms, pitch_ms, yaw_ms, knob_ms[2];
    uint16_t knob_anchor[2], knob_candidate_anchor[2];
    uint32_t context_id;
    int8_t pitch, roll, yaw;
    uint8_t active, ready, neutral, yaw_sent;
    uint16_t knob_bin[2], knob_candidate[2];
} rc_ui_t;
/* Display owner only. At most four actions per tick. Values must be calibrated.
 * Ground/safety/low-throttle gating is enforced here, independent of UI callbacks. */
unsigned rc_ui_update(rc_ui_t *ui, const rc_ui_frame_t *frame, const rc_ui_context_t *context,
                      rc_ui_action_t actions[4]);
#endif
