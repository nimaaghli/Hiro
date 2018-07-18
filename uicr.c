#include "nrf.h"
//#include "nrf51_bitfields.h"
#include "nrf_nvmc.h"
#include <string.h>

#include "uicr.h"

typedef struct verti_settings {
    bool sleepable;
    uint8_t uuid[16];
} verti_settings_t;

verti_settings_t settings;
void get_uicr(void) {
    memcpy(&settings, (verti_settings_t *)CONFIG_PAGE_ADDR, sizeof(settings));
    return;
}

void set_uicr(void) {
      nrf_nvmc_page_erase(CONFIG_PAGE_ADDR);
       nrf_nvmc_write_bytes(CONFIG_PAGE_ADDR, (uint8_t *)&settings,
                         sizeof(settings));
    return;
}

/* static int intpow(int a, int b) { */
/*   int out = 1; */
/*   while (b>0) { */
/*     out *= a; */
/*     b-=1; */
/*   } */
/*   return out; */
/* } */

/* char *myitoa(uint16_t i, char *buf, uint8_t base) { */
/*   char glyphs[16] = "0123456789abcdef"; */
/*   // find order (num digits) */
/*   uint8_t od = 1; */
/*   while ( i / intpow(base, od) ) od+=1; */
/*   od-=1; */
/*   for (uint8_t o = od; o>=0; o--) { */
/*     buf[od-o]=glyphs[i / intpow(base, o) % base]; */
/*   } */
/*   buf[od+1]=0; */
/*   return buf; */
/* } */
