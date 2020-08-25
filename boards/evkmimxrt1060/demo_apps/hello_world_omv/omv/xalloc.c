/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * Memory allocation functions.
 *
 */
#include "xalloc.h"
#include "stdbool.h"
#include "stdlib.h"
#include "string.h"
#ifdef WIN32
#define NORETURN 
#else 
#define NORETURN __attribute__((noreturn))
#endif
NORETURN void xalloc_fail()
{
//	printf("Out of Memory!!!");
#ifdef __CC_ARM
	__asm { BKPT 0xAB }
#else
	while (1);
#endif
}

// returns null pointer without error if size==0
void* xalloc(uint32_t size)
{
	void* mem = malloc(size);
	if (size && (mem == NULL)) {
		xalloc_fail();
	}
	return mem;
}

// returns null pointer without error if size==0
void* xalloc_try_alloc(uint32_t size)
{
	void* mem = malloc(size);
	if (size && (mem == NULL)) {
		return NULL;
	}
	return mem;
}

// returns null pointer without error if size==0
void* xalloc0(uint32_t size)
{
	void* mem = malloc(size);
	if (size && (mem == NULL)) {
		xalloc_fail();
	}
	memset(mem, 0, size);
	return mem;
}

// returns without error if mem==null
void xfree(void* mem)
{
	free(mem);
}

// returns null pointer without error if size==0
// allocs if mem==null and size!=0
// frees if mem!=null and size==0
void* xrealloc(void* mem, uint32_t size)
{
	mem = realloc(mem, size);
	if (size && (mem == NULL)) {
		xalloc_fail();
	}
	return mem;
}
