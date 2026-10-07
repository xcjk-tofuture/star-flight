#ifndef UAV_INTERNAL_FLASH_PORT_H
#define UAV_INTERNAL_FLASH_PORT_H
#include "nv_store.h"
#define UAV_PARAM_BANK_A 0x08040000u
#define UAV_PARAM_BANK_B 0x08060000u
#define UAV_PARAM_BANK_BYTES 0x20000u
int uav_internal_flash_probe(void);
nv_io_t uav_internal_flash_io(void);
#endif
