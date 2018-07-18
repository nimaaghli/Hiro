#ifndef UICR_H
#define UICR_H

#include "nrf.h"
#include <stdbool.h>



#define BOOTLOADER_SIZE (0x3C00 + 1024)
#define BOOTLOADER_PAGES (BOOTLOADER_SIZE / NRF_FICR->CODEPAGESIZE + 1)
#define CONFIG_PAGE_ADDR                                                       \
    ((NRF_FICR->CODESIZE - BOOTLOADER_PAGES) * NRF_FICR->CODEPAGESIZE)



void get_uicr(void);
void set_uicr(void);

// char *myitoa(uint16_t, char*, uint8_t);

#endif /* UICR_H */
