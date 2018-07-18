#define _GNU_SOURCE

#include "nrf.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "uicr.h"
#include <app_error.h>
#include <string.h>

#define RAND_LENGTH 16

uint8_t rand_buf[RAND_LENGTH] = {0};
char uuid_str_buf[RAND_LENGTH * 2 + 4] = {0};
typedef struct verti_settings {
    bool sleepable;
    uint8_t uuid[16];
} verti_settings_t;
verti_settings_t settings;

uint8_t *get_128bit_random(void) {
    // Pointer into rand_buf used for offset writes if entropy is low
    uint8_t *rand_i = rand_buf;
    uint8_t count = 0;
    uint32_t err_code;
    do {
        uint8_t avail = 0;
        err_code = sd_rand_application_bytes_available_get(&avail);
        APP_ERROR_CHECK(err_code);
        if (avail >= RAND_LENGTH - count) {
            sd_rand_application_vector_get(rand_i, RAND_LENGTH - count);
            count = RAND_LENGTH;
        } else {
            sd_rand_application_vector_get(rand_i, avail);
            rand_i += avail;
            count += avail;
        }
    } while (count < RAND_LENGTH);
    return rand_buf;
}

char *get_uuid(void) {
    /* Get 128 random from flash, generate new value if not present,
       return formatted RFC4122 uuid */
    char hexarray[] = "0123456789abcdef";
    if ((settings.uuid[6] & 0xf0) != 0x40) {
        // Not a verti uuid, probably first boot
        uint8_t *random = get_128bit_random();
        random[6] = 0x40 | (random[6] & 0xf);
        random[8] = 0x80 | (random[8] & 0x3f);
        memcpy(settings.uuid, random, RAND_LENGTH);
        //set_uicr();
    }
    for (uint8_t rand_i = 0, str_i = 0; rand_i < RAND_LENGTH; rand_i++) {
        uuid_str_buf[str_i++] = hexarray[(settings.uuid[rand_i] & 0xf0) >> 4];
        uuid_str_buf[str_i++] = hexarray[settings.uuid[rand_i] & 0x0f];
        if (str_i == 8 || str_i == 13 || str_i == 18 || str_i == 23) {
            uuid_str_buf[str_i++] = '-';
        }
    }
    return uuid_str_buf;
}
