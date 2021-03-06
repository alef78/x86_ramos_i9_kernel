/*
 * Support for Intel Camera Imaging ISP subsystem.
 *
 * Copyright (c) 2010 - 2014 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include "ia_css_circbuf.h"
/**********************************************************************
 *
 * Forward declarations.
 *
 **********************************************************************/
/**
 * @brief Read the oldest element from the circular buffer.
 * Read the oldest element WITHOUT checking whehter the
 * circular buffer is empty or not. The oldest element is
 * also removed out from the circular buffer.
 *
 * @param cb The pointer to the circular buffer.
 *
 * @return the oldest element.
 */
static inline ia_css_circbuf_elem_t
ia_css_circbuf_read(ia_css_circbuf_t *cb);

/**
 * @brief Shift a chunk of elements in the circular buffer.
 * A chunk of elements (i.e. the ones from the "start" position
 * to the "chunk_src" position) are shifted in the circular buffer,
 * along the direction of new elements coming.
 *
 * @param cb	     The pointer to the circular buffer.
 * @param chunk_src  The position at which the first element in the chunk is.
 * @param chunk_dest The position to which the first element in the chunk would be shift.
 */
static inline void ia_css_circbuf_shift_chunk(ia_css_circbuf_t *cb,
						   uint32_t chunk_src,
						   uint32_t chunk_dest);

/**
 * @brief Get the "val" field in the element.
 *
 * @param elem The pointer to the element.
 *
 * @return the "val" field.
 */
static inline uint32_t
ia_css_circbuf_elem_get_val(ia_css_circbuf_elem_t *elem);

/**********************************************************************
 *
 * Non-inline functions.
 *
 **********************************************************************/
/**
 * @brief Create the circular buffer.
 * Refer to "ia_css_circbuf.h" for details.
 */
void
ia_css_circbuf_create(ia_css_circbuf_t *cb,
			   ia_css_circbuf_elem_t *elems,
			   ia_css_circbuf_desc_t *desc)
{
	uint32_t i;

	OP___assert(desc);

	cb->desc = desc;
	/* Initialize to defaults */
	cb->desc->start = 0;
	cb->desc->end = 0;
	cb->desc->step = 0;

	for (i = 0; i < cb->desc->size; i++)
		ia_css_circbuf_elem_init(&elems[i]);

	cb->elems = elems;
}

/**
 * @brief Destroy the circular buffer.
 * Refer to "ia_css_circbuf.h" for details.
 */
void ia_css_circbuf_destroy(ia_css_circbuf_t *cb)
{
	cb->desc = NULL;

	cb->elems = NULL;
}

/**
 * @brief Pop a value out of the circular buffer.
 * Refer to "ia_css_circbuf.h" for details.
 */
uint32_t ia_css_circbuf_pop(ia_css_circbuf_t *cb)
{
	uint32_t ret;
	ia_css_circbuf_elem_t elem;

	if (ia_css_circbuf_is_empty(cb))
#ifdef __SP
		OP_std_break();
#else
		assert(0);
#endif
	/* read an element from the buffer */
	elem = ia_css_circbuf_read(cb);
	ret = ia_css_circbuf_elem_get_val(&elem);
	return ret;
}

/**
 * @brief Extract a value out of the circular buffer.
 * Refer to "ia_css_circbuf.h" for details.
 */
uint32_t ia_css_circbuf_extract(ia_css_circbuf_t *cb, int offset)
{
	int max_offset;
	uint32_t val;
	uint32_t pos;
	uint32_t src_pos;
	uint32_t dest_pos;

	/* get the maximum offest */
	max_offset = ia_css_circbuf_get_offset(cb, cb->desc->start, cb->desc->end);
	max_offset--;

	/*
	 * Step 1: When the target element is at the "start" position.
	 */
	if (offset == 0) {
		val = ia_css_circbuf_pop(cb);
		return val;
	}

	/*
	 * Step 2: When the target element is out of the range.
	 */
	if (offset > max_offset) {
		val = 0;
		return val;
	}

	/*
	 * Step 3: When the target element is between the "start" and
	 * "end" position.
	 */
	/* get the position of the target element */
	pos = ia_css_circbuf_get_pos_at_offset(cb, cb->desc->start, offset);

	/* get the value from the target element */
	val = ia_css_circbuf_elem_get_val(&cb->elems[pos]);

	/* shift the elements */
	src_pos = ia_css_circbuf_get_pos_at_offset(cb, pos, -1);
	dest_pos = pos;
	ia_css_circbuf_shift_chunk(cb, src_pos, dest_pos);

	return val;
}

/**
 * @brief Peek an element from the circular buffer.
 * Refer to "ia_css_circbuf.h" for details.
 */
uint32_t ia_css_circbuf_peek(ia_css_circbuf_t *cb, int offset)
{
	int pos;

	pos = ia_css_circbuf_get_pos_at_offset(cb, cb->desc->end, offset);

	/* get the value at the position */
	return cb->elems[pos].val;
}


/****************************************************************
 *
 * Inline functions.
 *
 ****************************************************************/
/**
 * @brief Get the "val" field in the element.
 * Refer to "Forward declarations" for details.
 */
static inline uint32_t
ia_css_circbuf_elem_get_val(ia_css_circbuf_elem_t *elem)
{
	return elem->val;
}

/**
 * @brief Read the oldest element from the circular buffer.
 * Refer to "Forward declarations" for details.
 */
static inline ia_css_circbuf_elem_t
ia_css_circbuf_read(ia_css_circbuf_t *cb)
{
	ia_css_circbuf_elem_t elem;

	/* get the element from the target position */
	elem = cb->elems[cb->desc->start];

	/* clear the target position */
	ia_css_circbuf_elem_init(&cb->elems[cb->desc->start]);

	/* adjust the "start" position */
	cb->desc->start = ia_css_circbuf_get_pos_at_offset(cb, cb->desc->start, 1);
	return elem;
}

/**
 * @brief Shift a chunk of elements in the circular buffer.
 * Refer to "Forward declarations" for details.
 */
static inline void
ia_css_circbuf_shift_chunk(ia_css_circbuf_t *cb,
				uint32_t chunk_src, uint32_t chunk_dest)
{
	int chunk_offset;
	int chunk_sz;
	int i;

	/* get the chunk offset and size */
	chunk_offset = ia_css_circbuf_get_offset(cb,
						      chunk_src, chunk_dest);
	chunk_sz = ia_css_circbuf_get_offset(cb, cb->desc->start, chunk_src) + 1;

	/* shift each element to its terminal position */
	for (i = 0; i < chunk_sz; i++) {

		/* copy the element from the source to the destination */
		ia_css_circbuf_elem_cpy(&cb->elems[chunk_src],
					     &cb->elems[chunk_dest]);

		/* adjust the source/terminal positions */
		chunk_src = ia_css_circbuf_get_pos_at_offset(cb, chunk_src, -1);
		chunk_dest = ia_css_circbuf_get_pos_at_offset(cb, chunk_dest, -1);

	}

	/* adjust the index "start" */
	cb->desc->start = ia_css_circbuf_get_pos_at_offset(cb, cb->desc->start, chunk_offset);
}

