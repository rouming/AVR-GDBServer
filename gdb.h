/******************************************************************************
 * Lightweight GDB server implementation for AVR MCU
 ******************************************************************************/
#ifndef GDB_H
#define GDB_H

#include <avr/io.h>
#include <stdint.h>

#define MAX_BREAKS       8
#define MAX_STEPI_BREAKS 3

/* must be in hex, and not fewer than 79 bytes,
   see gdb_read_registers for details */
#define MAX_BUFF   0x50

typedef uint8_t bool_t;
#define FALSE 0
#define TRUE 1

struct gdb_break
{
	uint16_t addr; /* in words */
	uint16_t opcode;
};

struct gdb_context
{
	struct {
		uint8_t stack_bottom;
		uint8_t r31;
		uint8_t r30;
		uint8_t r29;
		uint8_t r28;
		uint8_t r27;
		uint8_t r26;
		uint8_t r25;
		uint8_t r24;
		uint8_t r23;
		uint8_t r22;
		uint8_t r21;
		uint8_t r20;
		uint8_t r19;
		uint8_t r18;
		uint8_t r17;
		uint8_t r16;
		uint8_t r15;
		uint8_t r14;
		uint8_t r13;
		uint8_t r12;
		uint8_t r11;
		uint8_t r10;
		uint8_t r9;
		uint8_t r8;
		uint8_t r7;
		uint8_t r6;
		uint8_t r5;
		uint8_t r4;
		uint8_t r3;
		uint8_t r2;
		uint8_t r1;
		uint8_t sreg;
		uint8_t r0;
		uint8_t pc_h;
		uint8_t pc_l;
	} *regs;
	uint16_t sp;
	uint16_t pc;
	struct gdb_break breaks[MAX_BREAKS+MAX_STEPI_BREAKS];
	uint8_t breaks_cnt;
	bool_t  in_stepi;
	uint8_t buff[MAX_BUFF+1];
	uint8_t buff_sz;
};

/* Inits gdb server context, implicitly starts UART */
void gdb_init(struct gdb_context *ctx);


#endif //GDB_H
