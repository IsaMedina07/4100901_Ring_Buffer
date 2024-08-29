/*
 * ring_buffer.h
 *
 *  Created on: Aug 15, 2024
 *      Author: isabe
 */

#ifndef INC_RING_BUFFER_H_
#define INC_RING_BUFFER_H_

#include <stdint.h>

typedef struct ring_buffer_ {
	uint8_t *buffer;
	uint8_t head;
	uint8_t tail;
	uint8_t is_full;
	uint8_t capacity;
}ring_buffer_t;

void ring_buffer_reset();
uint8_t ring_buffer_size(ring_buffer_t *rb);
uint8_t ring_buffer_is_full(ring_buffer_t *rb);
uint8_t ring_buffer_is_empy(ring_buffer_t *rb);
uint8_t right_buffer_ID( uint8_t *data);

void ring_buffer_write(ring_buffer_t *rb, uint8_t data);
uint8_t ring_buffer_read(ring_buffer_t *rb, uint8_t *byte);

#endif /* INC_RING_BUFFER_H_ */
