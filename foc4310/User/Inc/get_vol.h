#ifndef _GET_VOL_H
#define _GET_VOL_H
#include "main.h"

extern float vbus_vol;
extern float ntc_vol;

void get_vbus_start_dma(void);
void update_vbus(void);
#endif
