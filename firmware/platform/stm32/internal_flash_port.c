#include "internal_flash_port.h"
#include "main.h"
#include "flight_snapshot.h"
#include <string.h>
extern char __firmware_flash_end;
static int bounds(unsigned bank, uint32_t off, size_t n) {
    return bank<2 && off<=UAV_PARAM_BANK_BYTES && n<=UAV_PARAM_BANK_BYTES-off;
}
int uav_internal_flash_probe(void) {
    return *(volatile uint16_t *)0x1fff7a22u>=512 &&
        (uintptr_t)&__firmware_flash_end<=UAV_PARAM_BANK_A ? 0:-1;
}
static uint32_t address(unsigned bank, uint32_t off) {
    return (bank ? UAV_PARAM_BANK_B:UAV_PARAM_BANK_A)+off;
}
static int read_bytes(void *ctx, unsigned bank, uint32_t off, uint8_t *b, size_t n) {
    (void)ctx;
    if (!bounds(bank,off,n)) return -1;
    memcpy(b,(const void *)(uintptr_t)address(bank,off),n);
    return 0;
}
static int locked(void) {
    flight_snapshot_t flight; flight_snapshot_read(&flight);
    return flight.state==0 && uav_internal_flash_probe()==0;
}
static int erase_bank(void *ctx, unsigned bank) {
    (void)ctx;
    if (bank>1 || !locked() || HAL_FLASH_Unlock()!=HAL_OK) return -1;
    FLASH_EraseInitTypeDef erase={.TypeErase=FLASH_TYPEERASE_SECTORS,
        .VoltageRange=FLASH_VOLTAGE_RANGE_3,.Sector=bank ? FLASH_SECTOR_7:FLASH_SECTOR_6,
        .NbSectors=1};
    uint32_t error;
    HAL_StatusTypeDef result=HAL_FLASHEx_Erase(&erase,&error);
    HAL_FLASH_Lock();
    return result==HAL_OK ? 0:-1;
}
static int program_bytes(void *ctx, unsigned bank, uint32_t off, const uint8_t *b, size_t n) {
    (void)ctx;
    if (!bounds(bank,off,n) || (off&3u) || (n&3u) || !locked() ||
        HAL_FLASH_Unlock()!=HAL_OK) return -1;
    int result=0;
    /* Polling HAL_FLASH_Program does not flush the F4 ART data cache. Reading
     * a just-programmed word could otherwise return a cached erased value. */
    uint32_t cache=FLASH->ACR & FLASH_ACR_DCEN;
    __HAL_FLASH_DATA_CACHE_DISABLE();
    __HAL_FLASH_DATA_CACHE_RESET();
    __DSB(); __ISB();
    for (size_t i=0;i<n;i+=4) {
        uint32_t word; memcpy(&word,b+i,4);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,address(bank,off+(uint32_t)i),word)!=HAL_OK ||
            *(volatile uint32_t *)(uintptr_t)address(bank,off+(uint32_t)i)!=word) {
            result=-1; break;
        }
    }
    __HAL_FLASH_DATA_CACHE_RESET();
    if (cache) __HAL_FLASH_DATA_CACHE_ENABLE();
    __DSB(); __ISB();
    HAL_FLASH_Lock();
    return result;
}
nv_io_t uav_internal_flash_io(void) {
    return (nv_io_t){NULL,UAV_PARAM_BANK_BYTES,read_bytes,erase_bank,program_bytes};
}
