#ifndef _GET_VOL_H
#define _GET_VOL_H
#include "main.h"

extern float vbus_vol;
extern float ntc_vol;

extern float Iu;
extern float Iv;
extern float Iw;

void get_vbus_start_dma(void);
void update_vbus(void);
void get_current_offset(void);
void get_current_start(void);
void get_current_update(void);
#endif
