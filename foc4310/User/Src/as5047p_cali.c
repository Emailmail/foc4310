#include "as5047p_cali.h"

uint32_t before_calicount = 10000;

uint32_t cali_count = 10000;
float cali_sum = 0.0f;
float cali_result = 0.0f;

void cali_update(void) {
    if(before_calicount > 0) {
        before_calicount--;
    } else {
        if(cali_count > 0) {
            cali_sum += raw_angle * 14.0f;
            cali_count--;
        } else {
            cali_result = cali_sum / 10000.0f;
        }
    }
}

