/******************************************************************************
 * Lightweight GDB server implementation for AVR MCU
 ******************************************************************************/
#ifndef GDB_H
#define GDB_H

enum gdb_interrupt_reason
{
	gdb_user_interrupt  = 0,
	gdb_breakpoint = 1
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
		enum gdb_interrupt_reason int_reason;
		void (*breakpoint_handler)();
	};
};

/* Inits gdb server context, implicitly starts UART */
void gdb_init(struct gdb_context *ctx);


#endif //GDB_H
