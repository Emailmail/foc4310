#include "link_data.h"
#include "stdlib.h"
#include "protocol.h"



void Link_init( Link *link){
	uint16_t i;
	for( i = 0 ; i < PACKAGE_SIZE ; i++ )
		link->data[i] = 0x00;
	link->head = &link->data[0];
	link->tail = &link->data[PACKAGE_SIZE-1];
	
	link->head_index = 0;
	link->read_index = 0;
	link->tail_index  = PACKAGE_SIZE-1;
}

uint8_t Link_Read_Data( Link *link , uint8_t isFrom_Head){
	if( isFrom_Head == ENABLE ){
		link->read_index = link->head_index;
	}
	else{
		if( link->read_index == PACKAGE_SIZE-1 )
			link->read_index = 0;
		else
			link->read_index += 1;
	}
	return link->data[link->read_index];
}

void Link_insert_ultra( Link *link , uint8_t data  ){
	if( link->tail_index == PACKAGE_SIZE-1 ){
		link->data[0] = data;
		link->tail_index = 0;
		link->head_index += 1;
		
		link->head = &link->data[link->head_index ];
		link->tail = &link->data[link->tail_index ];
	}
	else{
		link->tail_index += 1;
		link->data[link->tail_index] = data;
		if( link->head_index == PACKAGE_SIZE-1 )
			link->head_index = 0;
		else
			link->head_index += 1;
		link->head = &link->data[link->head_index ];
		link->tail = &link->data[link->tail_index ];	
	}
	
}