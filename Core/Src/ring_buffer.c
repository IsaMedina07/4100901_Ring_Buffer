/*
 * ring_buffer.c
 *
 *  Created on: Aug 15, 2024
 *      Author: isabe
 */

#include "ring_buffer.h"



/*#define capacity (10)
uint8_t ring_buffer[capacity];
uint8_t head_ptr;
uint8_t tail_ptr;
uint8_t is_full;*/

uint8_t numberID[] = {'1','0','0','6','9','6','4','0','9','8'};

// Clase 23/08/2024
void ring_buffer_init(ring_buffer_t *rb, uint8_t *mem_add, uint8_t cap){
	rb->buffer = mem_add;
	rb->capacity = cap;

	ring_buffer_reset(rb);
}

/**
 * @ brief Esta función escribe un dato en el buffer circular
 *
 * @ param data: Dato que se va a escribir
 *
 * @ retval Ninguno
 * */
void ring_buffer_write(ring_buffer_t *rb, uint8_t data){
		rb->buffer[rb->head] = data;
		rb->head++;

		if (rb->head >= rb->capacity){ // Si la cabeza llega al final de la memoria
			rb->head = 0;
		}

		if (rb->is_full != 0 ){ // Si se pierden datos viejos
			rb->tail += 1;
		}

		if (rb->tail >= rb->capacity){ // Si la cola llega al final de la memoria
			rb->tail = 0;
				}

		if (rb->head == rb->tail){ // Si la cabeza alcanza la cola
			rb->is_full = 1;
		}
}


/**
 * @ brief Esta función lee un dato en el buffer circular
 *
 * @ param data: Dirección de donde se va a escribir el dato
 *
 * @ retval 1: Hay datos disponibles, 0: No hay datos
 * */
uint8_t ring_buffer_read(ring_buffer_t *rb, uint8_t *byte){

	if( rb->is_full != 0 || rb->head != rb->tail ){
		  *byte = rb->buffer[rb->tail];
		  rb->tail += 1;

		  if (rb->tail >= rb->capacity) {
			  rb->tail = 0;
		  }
		  rb->is_full = 0;

		  return 1; // Buffer con datos
	}
	return 0; //Buffer vacío
}

void ring_buffer_reset(ring_buffer_t *rb){
	//ring_buffer[capacity] = 0;
	rb->head = 0;
	rb->tail = 0;
	rb->is_full = 0;
} ;

uint8_t ring_buffer_size(ring_buffer_t *rb){
	uint8_t size = 0;
	if(rb->head > rb->tail){
		size = rb->head - rb->tail;
	}
	else if(rb->head < rb->tail){
			size = rb->capacity - rb->tail + rb->head;
	}else{
		if(rb->is_full == 1){
			size = rb->capacity -1;
		}else{
			size = 0;
		}
	}

	return size;
};

uint8_t ring_buffer_is_full(ring_buffer_t *rb){
	return rb->is_full;
};

uint8_t ring_buffer_is_empy(ring_buffer_t *rb){
	if(rb->head == 0 && rb->tail == 0 && rb->is_full == 0){
		return 1;
	}else{
		return 0;
	}
};

uint8_t right_buffer_ID( uint8_t *data){

	 if ( memcmp(data, numberID, sizeof(numberID)) != 0 ) {
	        return 0;
	 } else {
	       return 1;
	 }
}
