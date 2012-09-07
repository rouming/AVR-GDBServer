/******************************************************************************
 * Lightweight GDB server implementation for AVR MCU
 ******************************************************************************/
#ifndef GDB_H
#define GDB_H

#include <avr/io.h>
#include <stdint.h>

#define MAX_BREAKS 8
/* must be in hex */
#define MAX_BUFF   0x40

#ifdef __AVR_ATmega16__
/* We use SBI as trap opcode
   1001 1010 AAAA Abbb, where AAAAA - mem, bbb - bit */
#define SBI_OPCODE     0x9a00
#define SBI_MEM_SHIFT  0x3
/* For trapping we use software interrupt INT0, for ATmega16 this is PD2 pin */
#define TRAP_OPCODE (SBI_OPCODE | ((_SFR_IO_ADDR(PORTD)) << SBI_MEM_SHIFT) | PD2)
#define DEBUG_BREAK asm volatile("sbi %0, %1" :: "I" (_SFR_IO_ADDR(PORTD)), "I"(PD2))
#else
#error Unsupported AVR architecture!
#endif

typedef uint8_t bool_t;
#define FALSE 0
#define TRUE 1

enum gdb_interrupt_reason
{
	gdb_user_interrupt   = 0,
	gdb_orig_breakpoint  = 1,
	gdb_stepi_breakpoint = 2
};

struct gdb_break
{
	uint16_t addr; /* in words */
	uint16_t opcode;
};

struct gdb_context
{
	union {
		uint8_t *stack;
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
			uint8_t ret_addr_h;
			uint8_t ret_addr_l;
		} *regs;
		uint16_t pc;
		enum gdb_interrupt_reason int_reason;
		struct gdb_break breaks[MAX_BREAKS];
		uint8_t breaks_cnt;
		bool_t in_stepi;
		uint8_t buff[MAX_BUFF];
		uint8_t buff_sz;
	};
};

/* Inits gdb server context, implicitly starts UART */
void gdb_init(struct gdb_context *ctx);


#endif //GDB_H
