#include "settings_record.h"
#include <string.h>
static const uint16_t minimum[UAV_SETTING_COUNT]={0,1,0,10,300,0,0,0};
static const uint16_t maximum[UAV_SETTING_COUNT]={2,7,1,100,550,5000,5000,2000};
static const uint16_t steps[UAV_SETTING_COUNT]={1,1,1,5,5,5,50,5};
unsigned uav_settings_min(unsigned f) { return f<UAV_SETTING_COUNT ? minimum[f]:0; }
unsigned uav_settings_max(unsigned f) { return f<UAV_SETTING_COUNT ? maximum[f]:0; }
unsigned uav_settings_step(unsigned f) { return f<UAV_SETTING_COUNT ? steps[f]:0; }
void uav_settings_defaults(uav_settings_values_t *s) {
    *s=(uav_settings_values_t){{UAV_SOUND_NORMAL,1,0,50,400,1000,200,0}};
}
void uav_settings_encode(uint8_t b[UAV_SETTINGS_BYTES], const uav_settings_values_t *s) {
    memset(b,0,UAV_SETTINGS_BYTES);
    for (unsigned i=0;i<4;i++) b[i]=(uint8_t)s->value[i];
    for (unsigned i=4;i<UAV_SETTING_COUNT;i++) {
        unsigned offset=4+(i-4)*2;
        b[offset]=(uint8_t)s->value[i]; b[offset+1]=(uint8_t)(s->value[i]>>8);
    }
}
int uav_settings_record_valid(const uint8_t *b, size_t n) {
    if (!b || n!=UAV_SETTINGS_BYTES) return 0;
    for (unsigned i=0;i<UAV_SETTING_COUNT;i++) {
        unsigned offset=i<4 ? i:4+(i-4)*2;
        unsigned value=i<4 ? b[offset]:(unsigned)b[offset]|((unsigned)b[offset+1]<<8);
        if (value<minimum[i] || value>maximum[i]) return 0;
    }
    for (unsigned i=12;i<UAV_SETTINGS_BYTES;i++) if (b[i]) return 0;
    return 1;
}
int uav_settings_decode(const uint8_t b[UAV_SETTINGS_BYTES], uav_settings_values_t *s) {
    if (!s || !uav_settings_record_valid(b,UAV_SETTINGS_BYTES)) return 0;
    for (unsigned i=0;i<UAV_SETTING_COUNT;i++) {
        unsigned offset=i<4 ? i:4+(i-4)*2;
        s->value[i]=(uint16_t)(i<4 ? b[offset]:(unsigned)b[offset]|((unsigned)b[offset+1]<<8));
    }
    return 1;
}
