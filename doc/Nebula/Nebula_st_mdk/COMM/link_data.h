#ifndef __LINK_DATA__H_
#define __LINK_DATA__H_

#include "main.h"


typedef struct Link
{
	uint8_t   data[24];
	uint8_t*  head;
	uint8_t*  tail;
	uint16_t  head_index;
	uint16_t  tail_index;
	uint16_t  read_index;
	
}Link;

void Link_init( Link *link);
uint8_t Link_Read_Data( Link *link , uint8_t isFrom_Head);
void Link_insert_ultra( Link *link , uint8_t data  );
#endif