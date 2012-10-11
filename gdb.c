/******************************************************************************
 * Lightweight embedded GDB server implementation for
 * 8-bit AVR MCU with 16 bit PC (i.e. 128kb max ROM)
 *
 * NOTE: this code is just a little bit experiment and written
 *       for fun, so I did not do any attempts to make it
 *       architecture independent.
 *
 * NOTE: we assume that all string constants that are being
 *       placed into flash, are located in section < 64K
 *       addresses, because I don't want to handle far pointers.
 *
 ******************************************************************************/
#include <avr/interrupt.h>
#include <avr/boot.h>
#include <avr/pgmspace.h>

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "gdb.h"

#if (SPM_PAGESIZE & (SPM_PAGESIZE - 1))
#error SPM_PAGESIZE is not power of two! Impossible!
#endif

#define SPM_PAGESIZE_W (SPM_PAGESIZE>>1)
#define ROUNDUP(x, s) (((x) + (s) - 1) & ~((s) - 1))
#define ROUNDDOWN(x, s) ((x) & ~((s) - 1))
#define ARRAY_SIZE(arr) (sizeof(arr)/sizeof((arr)[0]))
#define MIN(i1, i2) (i1 < i2 ? i1 : i2);

#define STR(s) #s
#define STR_VAL(s) STR(s)

#define MEM_SPACE_MASK 0x00ff0000
#define FLASH_OFFSET   0x00000000
#define SRAM_OFFSET    0x00800000

/* AVR puts garbage in hight bits on return address on stack.
   Mask them out */
#if defined(__AVR_ATmega16__)
#define RET_ADDR_MASK  0x1f
#else
#error Unsupported platform
#endif

/* Relative RJMP and RCALL 'k' address mask */
#define REL_K_MASK     0x0fff
#define REL_K_SHIFT    0

/* RET, RETI
   1001 0101 000N 1000
   PC(15:0) ← STACK */
#define RETn_OPCODE    0x9508
#define RETn_MASK      0xffef

/* CPSE
   0001 00rd dddd rrrr
   PC <- PC + 1, or PC + 2 or 3 */
#define CPSE_OPCODE    0x1000
#define CPSE_MASK      0xfc00

/* SBRC, SBRS
   1111 11Nr rrrr 0bbb
   PC <- PC + 1, or PC + 2 or 3 */
#define SBRn_OPCODE    0xfc00
#define SBRn_MASK      0xfc08

/* SBIC, SBIS
   1001 10N1 AAAA Abbb
   PC <- PC + 1, or PC + 2 or 3 */
#define SBIn_OPCODE    0x9900
#define SBIn_MASK      0xfd00

/* BREQ, BRNE, BRCS, BRCC, BRSH, BRLO, BRMI, BRPL, BRGE,
   BRLT, BRHS, BRHC, BRTS, BRTC, BRVS, BRVC, BRIE, BRID
   1111 0Nkk kkkk kNNN
   PC <- PC + 1, or PC + k + 1 */
#define BRCH_OPCODE    0xf000
#define BRCH_MASK      0xf800

/* 'k' address mask for all branch opcodes */
#define BRCH_K_MASK    0x03f8
#define BRCH_K_SHIFT   3

/* ICALL
   1001 0101 0000 1001
   PC(15:0) ← Z(15:0) */
#define ICALL_OPCODE   0x9509

/* RCALL
   1101 kkkk kkkk kkkk */
#define RCALL_OPCODE   0xd000
#define RCALL_MASK     0xf000

/* EICALL
   1001 0101 0001 1010
   PC(15:0) ← Z(15:0)
   PC(21:16) ← EIND (TODO) */
#define EICALL_OPCODE  0x951a

/* CALL
   1001 010k kkkk 111k
   kkkk kkkk kkkk kkkk */
#define CALL_OPCODE    0x940e
#define CALL_MASK      0xfe0e

/* IJMP
   1001 0100 0000 1001
   PC(15:0) ← Z(15:0) */
#define IJMP_OPCODE    0x9409

/* RJMP
   1100 kkkk kkkk kkkk */
#define RJMP_OPCODE    0xc000
#define RJMP_MASK      0xf000 /* similar to RCALL_MASK */

/* EIJMP
   1001 0100 0001 1001
   PC(15:0) ← Z(15:0)
   PC(21:16) ← EIND (TODO) */
#define EIJMP_OPCODE   0x9419

/* JMP
   1001 010k kkkk 110k
   kkkk kkkk kkkk kkkk */
#define JMP_OPCODE     0x940c
#define JMP_MASK       0xfe0e /* similar to CALL_MASK */

/* LDS
   1001 000d dddd 0000
   kkkk kkkk kkkk kkkk */
#define LDS_OPCODE     0x9000
#define LDS_MASK       0xfe0f

/* STS
   1001 001d dddd 0000
   kkkk kkkk kkkk kkkk */
#define STS_OPCODE     0x9200
#define STS_MASK       0xfe0f /* similar to LDS_MASK */

/* For trapping we use RJMP on itself, i.e. endless loop,
   1100 kkkk kkkk kkkk, where 'k' is a -1 in words */
#define TRAP_OPCODE 0xcfff

/* This are similar to unix signal numbers.
   See signum.h on unix systems for the values. */
#define GDB_SIGINT  2      /* Interrupt (ANSI). */
#define GDB_SIGTRAP 5      /* Trace trap (POSIX). */

#define GDB_SAVE_CONTEXT()									\
	asm volatile (	"push	r0						\n\t"	\
					"in		r0, __SREG__			\n\t"	\
					"cli							\n\t"	\
					"push	r0						\n\t"	\
					"push	r1						\n\t"	\
					"clr	r1						\n\t"	\
					"push	r2						\n\t"	\
					"push	r3						\n\t"	\
					"push	r4						\n\t"	\
					"push	r5						\n\t"	\
					"push	r6						\n\t"	\
					"push	r7						\n\t"	\
					"push	r8						\n\t"	\
					"push	r9						\n\t"	\
					"push	r10						\n\t"	\
					"push	r11						\n\t"	\
					"push	r12						\n\t"	\
					"push	r13						\n\t"	\
					"push	r14						\n\t"	\
					"push	r15						\n\t"	\
					"push	r16						\n\t"	\
					"push	r17						\n\t"	\
					"push	r18						\n\t"	\
					"push	r19						\n\t"	\
					"push	r20						\n\t"	\
					"push	r21						\n\t"	\
					"push	r22						\n\t"	\
					"push	r23						\n\t"	\
					"push	r24						\n\t"	\
					"push	r25						\n\t"	\
					"push	r26						\n\t"	\
					"push	r27						\n\t"	\
					"push	r28						\n\t"	\
					"push	r29						\n\t"	\
					"push	r30						\n\t"	\
					"push	r31						\n\t"	\
					"lds	r26, gdb_ctx			\n\t"	\
					"lds	r27, gdb_ctx + 1		\n\t"	\
					"in		r0, __SP_L__			\n\t"	\
					"st		x+, r0					\n\t"	\
					"in		r0, __SP_H__			\n\t"	\
					"st		x+, r0					\n\t"	\
					)

#define GDB_RESTORE_CONTEXT()								\
	asm volatile (	"lds	r26, gdb_ctx			\n\t"	\
					"lds	r27, gdb_ctx + 1		\n\t"	\
					"ld		r28, x+					\n\t"	\
					"out	__SP_L__, r28			\n\t"	\
					"ld		r29, x+					\n\t"	\
					"out	__SP_H__, r29			\n\t"	\
					"pop	r31						\n\t"	\
					"pop	r30						\n\t"	\
					"pop	r29						\n\t"	\
					"pop	r28						\n\t"	\
					"pop	r27						\n\t"	\
					"pop	r26						\n\t"	\
					"pop	r25						\n\t"	\
					"pop	r24						\n\t"	\
					"pop	r23						\n\t"	\
					"pop	r22						\n\t"	\
					"pop	r21						\n\t"	\
					"pop	r20						\n\t"	\
					"pop	r19						\n\t"	\
					"pop	r18						\n\t"	\
					"pop	r17						\n\t"	\
					"pop	r16						\n\t"	\
					"pop	r15						\n\t"	\
					"pop	r14						\n\t"	\
					"pop	r13						\n\t"	\
					"pop	r12						\n\t"	\
					"pop	r11						\n\t"	\
					"pop	r10						\n\t"	\
					"pop	r9						\n\t"	\
					"pop	r8						\n\t"	\
					"pop	r7						\n\t"	\
					"pop	r6						\n\t"	\
					"pop	r5						\n\t"	\
					"pop	r4						\n\t"	\
					"pop	r3						\n\t"	\
					"pop	r2						\n\t"	\
					"pop	r1						\n\t"	\
					"pop	r0						\n\t"	\
					"out	__SREG__, r0			\n\t"	\
					"pop	r0						\n\t"	\
					)


static const uint8_t *gdb_target_desc;
static uint8_t gdb_target_desc_len;

static const uint8_t *gdb_pkt_sz_desc;
static uint8_t gdb_pkt_sz_desc_len;

static struct gdb_context *gdb_ctx;

static void gdb_trap(void);
static struct gdb_break *gdb_find_break(uint16_t rom_addr);
static void gdb_remove_breakpoint_ptr(struct gdb_break *breakp);
static void gdb_send_state(uint8_t signo);

/* Convert number 0-15 to hex */
#define nib2hex(i) (uint8_t)((i) > 9 ? 'a' - 10 + (i) : '0' + (i))

/* Convert a hexidecimal digit to a 4 bit nibble. */
static uint8_t hex2nib(uint8_t hex)
{
	if (hex >= 'A' && hex <= 'F')
		return 10 + (hex - 'A');
	else if (hex >= 'a' && hex <= 'f')
		return 10 + (hex - 'a');
	else if (hex >= '0' && hex <= '9')
		return hex - '0';

	return 0xff;
}

static uint8_t parse_hex(const uint8_t *buff, uint32_t *hex)
{
	uint8_t nib, len;
	for (*hex = 0, len = 0; (nib = hex2nib(buff[len])) != 0xff; ++len)
		*hex = (*hex << 4) + nib;
	return len;
}

static uint16_t safe_pgm_read_word(uint32_t rom_addr_b)
{
#ifdef pgm_read_word_far
	if (rom_addr_b >= (1l<<16))
		return pgm_read_word_far(rom_addr_b);
	else
#endif
		return pgm_read_word(rom_addr_b);
}

static uint8_t safe_pgm_read_byte(uint32_t rom_addr_b)
{
#ifdef pgm_read_byte_far
	if (rom_addr_b >= (1l<<16))
		return pgm_read_byte_far(rom_addr_b);
	else
#endif
		return pgm_read_byte(rom_addr_b);
}

/* rom_addr - in words, sz - in bytes and must be multiple of two.
   NOTE: interrupts must be disabled before call of this func */
__attribute__ ((section(".nrww"),noinline))
static void safe_pgm_write(const void *ram_addr,
						   uint16_t rom_addr,
						   uint16_t sz)
{
	uint16_t *ram = (uint16_t*)ram_addr;

	/* Sz must be valid and be multiple of two */
	if (!sz || sz & 1)
		return;

	/* Avoid conflicts with EEPROM */
	eeprom_busy_wait();

	/* to words */
	sz >>= 1;

	for (uint16_t page = ROUNDDOWN(rom_addr, SPM_PAGESIZE_W),
		 end_page = ROUNDUP(rom_addr + sz, SPM_PAGESIZE_W),
		 off = rom_addr % SPM_PAGESIZE_W;
		 page < end_page;
		 page += SPM_PAGESIZE_W, off = 0) {

		/* page to bytes */
		uint32_t page_b = (uint32_t)page << 1;

		/* Fill temporary page */
		for (uint16_t page_off = 0;
			 page_off < SPM_PAGESIZE_W;
			 ++page_off) {
			/* to bytes */
			uint32_t rom_addr_b = ((uint32_t)page + page_off) << 1;

			/* Fill with word from ram */
			if (page_off == off) {
				boot_page_fill(rom_addr_b,  *ram);
				if (sz -= 1) {
					off += 1;
					ram += 1;
				}
			}
			/* Fill with word from flash */
			else
				boot_page_fill(rom_addr_b, safe_pgm_read_word(rom_addr_b));
		}

		/* Erase page and wait until done. */
		boot_page_erase(page_b);
		boot_spm_busy_wait();

		/* Write page and wait until done. */
		boot_page_write(page_b);
		boot_spm_busy_wait();
	}

	/* Reenable RWW-section again to jump to it */
	boot_rww_enable ();
}

/******************************************************************************/

void init_timer1(void)
{
#define TIMER1_RATE 1000

#if defined(__AVR_ATmega16__)
	/* Set CTC mode */
	TCCR1B |= (1 << WGM12);
	/* No prescaler */
	TCCR1B |= (1 << CS10);
	/* Set the compare register */
	OCR1A = F_CPU / TIMER1_RATE - 1;
	/* Enable Output Compare Match Interrupt */
	TIMSK |= (1 << OCIE1A);
#else
#error Unsupported AVR device
#endif
}

void init_uart(void)
{
#define BAUD_RATE 9600

#if defined(__AVR_ATmega16__)
	uint16_t ubrr = F_CPU / 16 / BAUD_RATE - 1;

	/* Disable uart rx/tx first */
	UCSRB = 0;

	/* Set baud rate */
	UBRRH = (unsigned char)(ubrr>>8);
	UBRRL = (unsigned char)ubrr;
	/* Set frame format: 8data, 1stop bit */
	UCSRC = (1<<URSEL)|(3<<UCSZ0);
	/* Enable receiver, transmitter and RX interrupt */
	UCSRB = (1<<RXEN)|(1<<TXEN)|(1<<RXCIE);
#else
#error Unsupported AVR device
#endif
}

ISR(TIMER1_COMPA_vect, ISR_NAKED)
{
	static uint16_t s_overflows = 0;

	/* Context save must be the first */
	GDB_SAVE_CONTEXT();

	/* We do all heavy checks every second */
	if (++s_overflows != TIMER1_RATE)
		goto out;
	s_overflows = 0;

	gdb_ctx->regs->pc_h &= RET_ADDR_MASK;
	/* Advance to application stack on 32 registers, SREG and 16-bit PC.
	   TODO: 24-bit PC unsupported */
	gdb_ctx->sp = (uintptr_t)gdb_ctx->regs + 35;
	gdb_ctx->pc = (gdb_ctx->regs->pc_h << 8) |
				  (gdb_ctx->regs->pc_l);

	/* Check breakpoint */
	for (uint8_t i = 0; i < ARRAY_SIZE(gdb_ctx->breaks); ++i)
		if (gdb_ctx->pc == gdb_ctx->breaks[i].addr)
			goto trap;

	/* Nothing */
	goto out;

trap:
	/* Set correct interrupt reason */
	if (gdb_ctx->in_stepi) {
		/* Remove all previous stepi breaks */
		for (uint8_t i = 0;
			 i < ARRAY_SIZE(gdb_ctx->breaks) && gdb_ctx->breaks_cnt; ++i)
			if (gdb_ctx->breaks[i].addr)
				gdb_remove_breakpoint_ptr(&gdb_ctx->breaks[i]);
		gdb_ctx->in_stepi = FALSE;
	}

	gdb_send_state(GDB_SIGTRAP);
	gdb_trap();

out:
	GDB_RESTORE_CONTEXT();
	asm volatile ("reti \n\t");
}

ISR(USART_RXC_vect, ISR_NAKED)
{
	GDB_SAVE_CONTEXT();
	gdb_ctx->regs->pc_h &= RET_ADDR_MASK;
	/* Advance to application stack on 32 registers, SREG and 16-bit PC.
	   TODO: 24-bit PC unsupported */
	gdb_ctx->sp = (uintptr_t)gdb_ctx->regs + 35;
	gdb_ctx->pc = (gdb_ctx->regs->pc_h << 8) |
				  (gdb_ctx->regs->pc_l);
	gdb_trap();
	GDB_RESTORE_CONTEXT();
	asm volatile ("reti \n\t");
}

/******************************************************************************/

void gdb_init(struct gdb_context *ctx)
{
	/* Init static ptrs to program mem */
	gdb_target_desc = (uint8_t*)PSTR(
		"<?xml version=\"1.0\"?>\n"
		"<!DOCTYPE target SYSTEM \"gdb-target.dtd\">\n"
		"<target version=\"1.0\">\n"
		"  <architecture>avr</architecture>\n"
		"</target>\n");
	gdb_target_desc_len = strlen_PF((uintptr_t)gdb_target_desc);

	/* NOTE: PacketSize=0xXX, i.e. MAX_BUFF must be in hex */
	gdb_pkt_sz_desc = (uint8_t*)PSTR(
		"PacketSize=" STR_VAL(MAX_BUFF) ";qXfer:features:read+");
	gdb_pkt_sz_desc_len = strlen_PF((uintptr_t)gdb_pkt_sz_desc);

	/* Init gdb context */
	gdb_ctx = ctx;
	gdb_ctx->sp = 0;
	gdb_ctx->breaks_cnt = 0;
	gdb_ctx->buff_sz = 0;
	gdb_ctx->in_stepi = FALSE;

	/* Init breaks */
	memset(gdb_ctx->breaks, 0, sizeof(gdb_ctx->breaks));

	init_timer1();
	init_uart();
}

static void gdb_send_byte(uint8_t b)
{
#if defined (__AVR_ATmega16__)
	/* Wait for empty transmit buffer */
	while (!(UCSRA & (1<<UDRE)))
		;
	UDR = b;
#else
#error Unsupported AVR device
#endif
}

static uint8_t gdb_read_byte(void)
{
#if defined (__AVR_ATmega16__)
	/* Wait for data to be received */
	while (!(UCSRA & (1<<RXC)))
		/* TODO: watchdog reset */
		;
	return UDR;
#else
#error Unsupported AVR device
#endif
}

struct buff_vec
{
	const uint8_t *buff;
	uint8_t off;
	uint8_t sz;
	bool_t in_pgm;
};

static void gdb_send_buff_vec(const struct buff_vec *vec, uint8_t cnt)
{
	uint8_t sum = 0;
	uint8_t b;

	gdb_send_byte('$');

	for (uint8_t ind = 0; ind < cnt; ++ind) {
		for (uint8_t i = 0; i < vec[ind].sz; ++i) {
			if (vec[ind].in_pgm)
				/* NOTE: rom buffer must be located in < 64K address section */
				b = pgm_read_byte(&vec[ind].buff[i + vec[ind].off]);
			else
				b = vec[ind].buff[i + vec[ind].off];

			sum += b;
			gdb_send_byte(b);
		}
	}

	gdb_send_byte('#');
	gdb_send_byte(nib2hex((sum >> 4) & 0xf));
	gdb_send_byte(nib2hex(sum & 0xf));
}

static void gdb_send_buff(const uint8_t *buff, uint8_t off,
						  uint8_t sz, bool_t in_pgm)
{
	struct buff_vec vec[] = { {.buff = buff,
							   .off = off,
							   .sz = sz,
							   .in_pgm = in_pgm} };
	gdb_send_buff_vec(vec, 1);
}

static void gdb_send_reply(const char *reply)
{
	uint8_t len = strlen(reply);
	gdb_ctx->buff_sz = MIN(len, sizeof(gdb_ctx->buff));
	memcpy(gdb_ctx->buff, reply, gdb_ctx->buff_sz);
	gdb_send_buff(gdb_ctx->buff, 0, gdb_ctx->buff_sz, FALSE);
}

static void gdb_send_state(uint8_t signo)
{
	uint32_t pc = (uint32_t)gdb_ctx->pc << 1;

	/* thread is always 1 */
	memcpy_P(gdb_ctx->buff,
			 PSTR("TXX20:XX;21:XXXX;22:XXXXXXXX;thread:1;"),
			 38);
	gdb_ctx->buff_sz = 38;

	/* signo */
	gdb_ctx->buff[1] = nib2hex((signo >> 4)  & 0xf);
	gdb_ctx->buff[2] = nib2hex(signo & 0xf);

	/* sreg */
	gdb_ctx->buff[6] = nib2hex((gdb_ctx->regs->sreg >> 4)  & 0xf);
	gdb_ctx->buff[7] = nib2hex(gdb_ctx->regs->sreg & 0xf);

	/* sp */
	gdb_ctx->buff[12] = nib2hex((gdb_ctx->sp >> 4)  & 0xf);
	gdb_ctx->buff[13] = nib2hex((gdb_ctx->sp >> 0)  & 0xf);
	gdb_ctx->buff[14] = nib2hex((gdb_ctx->sp >> 12) & 0xf);
	gdb_ctx->buff[15] = nib2hex((gdb_ctx->sp >> 8)  & 0xf);

	/* pc */
	gdb_ctx->buff[20] = nib2hex((pc >> 4)  & 0xf);
	gdb_ctx->buff[21] = nib2hex((pc >> 0)  & 0xf);
	gdb_ctx->buff[22] = nib2hex((pc >> 12) & 0xf);
	gdb_ctx->buff[23] = nib2hex((pc >> 8)  & 0xf);
	gdb_ctx->buff[24] = '0'; /* TODO: 22-bits not supported now */
	gdb_ctx->buff[25] = nib2hex((pc >> 16) & 0xf);
	gdb_ctx->buff[26] = '0'; /* gdb wants 32-bit value, send 0 */
	gdb_ctx->buff[27] = '0'; /* gdb wants 32-bit value, send 0 */

	/* not in hex, send from ram */
	gdb_send_buff(gdb_ctx->buff, 0, gdb_ctx->buff_sz, FALSE);
}

/* GDB needs the 32 8-bit, gpw registers (r00 - r31), the
   8-bit SREG, the 16-bit SP (stack pointer) and the 32-bit PC
   (program counter). Thus need to send a reply with
   r00, r01, ..., r31, SREG, SPL, SPH, PCL, PCH,
   low bytes before high since AVR is little endian.
   This routine requires (32 gpwr, SREG, SP, PC) * 2 hex bytes
   space of buffer, i.e. min (32 + 1 + 2 + 4) * 2 = 78 */
static void gdb_read_registers(void)
{
	uint32_t pc = (uint32_t)gdb_ctx->pc << 1;
	uint8_t i = 0;

	/* send r0 */
	gdb_ctx->buff[i++] = nib2hex((gdb_ctx->regs->r0 >> 4) & 0xf);
	gdb_ctx->buff[i++] = nib2hex((gdb_ctx->regs->r0 >> 0) & 0xf);

	/* send r1..r31 */
	for (uint8_t *ptr = &gdb_ctx->regs->r1; i < 32*2; --ptr) {
		gdb_ctx->buff[i++] = nib2hex((*ptr >> 4) & 0xf);
		gdb_ctx->buff[i++] = nib2hex((*ptr >> 0) & 0xf);
	}

	/* send SREG as 32 register */
	gdb_ctx->buff[i++] = nib2hex((gdb_ctx->regs->sreg >> 4) & 0xf);
	gdb_ctx->buff[i++] = nib2hex((gdb_ctx->regs->sreg >> 0) & 0xf);

	/* send SP as 33 register */
	gdb_ctx->buff[i++] = nib2hex((gdb_ctx->sp >> 4)  & 0xf);
	gdb_ctx->buff[i++] = nib2hex((gdb_ctx->sp >> 0)  & 0xf);
	gdb_ctx->buff[i++] = nib2hex((gdb_ctx->sp >> 12) & 0xf);
	gdb_ctx->buff[i++] = nib2hex((gdb_ctx->sp >> 8)  & 0xf);

	/* send PC as 34 register
	   gdb stores PC in a 32 bit value.
	   gdb thinks PC is bytes into flash, not in words. */
	gdb_ctx->buff[i++] = nib2hex((pc >> 4)  & 0xf);
	gdb_ctx->buff[i++] = nib2hex((pc >> 0)  & 0xf);
	gdb_ctx->buff[i++] = nib2hex((pc >> 12) & 0xf);
	gdb_ctx->buff[i++] = nib2hex((pc >> 8)  & 0xf);
	gdb_ctx->buff[i++] = '0'; /* TODO: 22-bits not supported now */
	gdb_ctx->buff[i++] = nib2hex((pc >> 16) & 0xf);
	gdb_ctx->buff[i++] = '0'; /* gdb wants 32-bit value, send 0 */
	gdb_ctx->buff[i++] = '0'; /* gdb wants 32-bit value, send 0 */

	gdb_ctx->buff_sz = i;
	gdb_send_buff(gdb_ctx->buff, 0, gdb_ctx->buff_sz, FALSE);
}

static void gdb_write_registers(const uint8_t *buff)
{
	uint32_t pc;

	/* receive r0 */
	gdb_ctx->regs->r0  = hex2nib(*buff++) << 4;
	gdb_ctx->regs->r0 |= hex2nib(*buff++);

	/* receive r1..r31 */
	for (uint8_t *ptr = &gdb_ctx->regs->r1;
		 ptr != &gdb_ctx->regs->stack_bottom; --ptr) {
		*ptr  = hex2nib(*buff++) << 4;
		*ptr |= hex2nib(*buff++);
	}

	/* receive SREG as 32 register */
	gdb_ctx->regs->sreg  = hex2nib(*buff++) << 4;
	gdb_ctx->regs->sreg |= hex2nib(*buff++);

	/* receive SP as 33 register */
	gdb_ctx->sp  = hex2nib(*buff++) << 4;
	gdb_ctx->sp |= hex2nib(*buff++);
	gdb_ctx->sp |= hex2nib(*buff++) << 12;
	gdb_ctx->sp |= hex2nib(*buff++) << 8;

	/* receive PC as 34 register
	   gdb stores PC in a 32 bit value.
	   gdb thinks PC is bytes into flash, not in words. */
	pc  = hex2nib(*buff++) << 4;
	pc |= hex2nib(*buff++);
	pc |= hex2nib(*buff++) << 12;
	pc |= hex2nib(*buff++) << 8;
	pc |= (uint32_t)hex2nib(*buff++) << 20;
	pc |= (uint32_t)hex2nib(*buff++) << 16;
	pc |= (uint32_t)hex2nib(*buff++) << 28;
	pc |= (uint32_t)hex2nib(*buff++) << 24;
	gdb_ctx->pc = pc >> 1;

	gdb_send_reply("OK");
}

static void gdb_read_register(const uint8_t *buff)
{
	uint32_t reg;
	uint8_t i = 0;

	parse_hex(buff, &reg);
	/* r0 */
	if (reg == 0) {
		gdb_ctx->buff[i++] = nib2hex((gdb_ctx->regs->r0 >> 4) & 0xf);
		gdb_ctx->buff[i++] = nib2hex((gdb_ctx->regs->r0 >> 0) & 0xf);
	}
	/* r1..r31 */
	if (reg > 0 && reg < 32) {
		uint8_t *ptr = &gdb_ctx->regs->r1 - (reg - 1);
		gdb_ctx->buff[i++] = nib2hex((*ptr >> 4) & 0xf);
		gdb_ctx->buff[i++] = nib2hex((*ptr >> 0) & 0xf);
	}
	/* sreg */
	else if (reg == 32) {
		gdb_ctx->buff[i++] = nib2hex((gdb_ctx->regs->sreg >> 4) & 0xf);
		gdb_ctx->buff[i++] = nib2hex((gdb_ctx->regs->sreg >> 0) & 0xf);
	}
	/* sp */
	else if (reg == 33) {
		gdb_ctx->buff[i++] = nib2hex((gdb_ctx->sp >> 4)  & 0xf);
		gdb_ctx->buff[i++] = nib2hex((gdb_ctx->sp >> 0)  & 0xf);
		gdb_ctx->buff[i++] = nib2hex((gdb_ctx->sp >> 12) & 0xf);
		gdb_ctx->buff[i++] = nib2hex((gdb_ctx->sp >> 8)  & 0xf);
	}
	/* pc */
	else if (reg == 34) {
		uint32_t pc = (uint32_t)gdb_ctx->pc << 1;
		gdb_ctx->buff[i++] = nib2hex((pc >> 4)  & 0xf);
		gdb_ctx->buff[i++] = nib2hex((pc >> 0)  & 0xf);
		gdb_ctx->buff[i++] = nib2hex((pc >> 12) & 0xf);
		gdb_ctx->buff[i++] = nib2hex((pc >> 8)  & 0xf);
		gdb_ctx->buff[i++] = '0'; /* TODO: 22-bits not supported now */
		gdb_ctx->buff[i++] = nib2hex((pc >> 16) & 0xf);
		gdb_ctx->buff[i++] = '0'; /* gdb wants 32-bit value, send 0 */
		gdb_ctx->buff[i++] = '0'; /* gdb wants 32-bit value, send 0 */
	}
	/* error */
	else {
		gdb_send_reply("E00");
		return;
	}

	gdb_ctx->buff_sz = i;
	gdb_send_buff(gdb_ctx->buff, 0, gdb_ctx->buff_sz, FALSE);
}

static void gdb_write_register(const uint8_t *buff)
{
	uint32_t reg;
	uint8_t len;

	len = parse_hex(buff, &reg);
	buff += len + 1;
	/* r0 */
	if (reg == 0) {
		gdb_ctx->regs->r0  = hex2nib(*buff++) << 4;
		gdb_ctx->regs->r0 |= hex2nib(*buff++);
	}
	/* r1..r31 */
	if (reg > 0 && reg < 32) {
		uint8_t *ptr = &gdb_ctx->regs->r1 - (reg - 1);
		*ptr  = hex2nib(*buff++) << 4;
		*ptr |= hex2nib(*buff++);
	}
	/* sreg */
	else if (reg == 32) {
		gdb_ctx->regs->sreg  = hex2nib(*buff++) << 4;
		gdb_ctx->regs->sreg |= hex2nib(*buff++);
	}
	/* sp */
	else if (reg == 33) {
		gdb_ctx->sp  = hex2nib(*buff++) << 4;
		gdb_ctx->sp |= hex2nib(*buff++);
		gdb_ctx->sp |= hex2nib(*buff++) << 12;
		gdb_ctx->sp |= hex2nib(*buff++) << 8;
	}
	/* pc */
	else if (reg == 34) {
		uint32_t pc;
		pc  = hex2nib(*buff++) << 4;
		pc |= hex2nib(*buff++);
		pc |= hex2nib(*buff++) << 12;
		pc |= hex2nib(*buff++) << 8;
		pc |= (uint32_t)hex2nib(*buff++) << 20;
		pc |= (uint32_t)hex2nib(*buff++) << 16;
		pc |= (uint32_t)hex2nib(*buff++) << 28;
		pc |= (uint32_t)hex2nib(*buff++) << 24;
		gdb_ctx->pc = pc >> 1;
	}
	/* error */
	else {
		gdb_send_reply("E00");
		return;
	}

	gdb_send_reply("OK");
}

static void gdb_read_memory(const uint8_t *buff)
{
	uint32_t addr, sz;

	buff += parse_hex(buff, &addr);
	/* skip 'xxx,' */
	parse_hex(buff + 1, &sz);

	if ((addr & MEM_SPACE_MASK) == SRAM_OFFSET) {
		addr &= ~MEM_SPACE_MASK;
		uint8_t *ptr = (uint8_t*)(uintptr_t)addr;
		for (uint8_t i = 0; i < sz; ++i) {
			uint8_t b = ptr[i];
			/* XXX: this is ugly kludge, but what can I do?
					AVR puts return address on stack with garbage in high
					bits (they say you should mask out them, see Stack Pointer
					section at every AVR datasheet), but how can I understand
					that this word is ret address? To have valid backtrace in
					gdb, I'am required to mask every word, which address belongs
					to stack. */
			if (i == 0 && sz == 2 && addr >= gdb_ctx->sp)
				b &= RET_ADDR_MASK;
			gdb_ctx->buff[i*2 + 0] = nib2hex(b >> 4);
			gdb_ctx->buff[i*2 + 1] = nib2hex(b & 0xf);
		}
	}
	else if ((addr & MEM_SPACE_MASK) == FLASH_OFFSET){
		addr &= ~MEM_SPACE_MASK;
		for (uint8_t i = 0; i < sz; ++i) {
			uint8_t byte = safe_pgm_read_byte(addr + i);
			gdb_ctx->buff[i*2 + 0] = nib2hex(byte >> 4);
			gdb_ctx->buff[i*2 + 1] = nib2hex(byte & 0xf);
		}
	}
	else {
		/* posix EIO error */
		gdb_send_reply("E05");
		return;
	}
	gdb_ctx->buff_sz = sz * 2;
	gdb_send_buff(gdb_ctx->buff, 0, gdb_ctx->buff_sz, FALSE);
}

static void gdb_write_memory(const uint8_t *buff)
{
	uint32_t addr, sz;

	buff += parse_hex(buff, &addr);
	/* skip 'xxx,' */
	buff += parse_hex(buff + 1, &sz);
	/* skip , and : delimiters */
	buff += 2;

	if ((addr & MEM_SPACE_MASK) == SRAM_OFFSET) {
		addr &= ~MEM_SPACE_MASK;
		uint8_t *ptr = (uint8_t*)(uintptr_t)addr;
		for (uint8_t i = 0; i < sz; ++i) {
			ptr[i]  = hex2nib(*buff++) << 4;
			ptr[i] |= hex2nib(*buff++);
		}
	}
	else if ((addr & MEM_SPACE_MASK) == FLASH_OFFSET){
		addr &= ~MEM_SPACE_MASK;
		/* to words */
		addr >>= 1;
		/* we assume sz is always multiple of two, i.e. write words */
		for (uint8_t i = 0; i < sz/2; ++i) {
			uint16_t word;
			word  = hex2nib(*buff++) << 4;
			word |= hex2nib(*buff++);
			word |= hex2nib(*buff++) << 12;
			word |= hex2nib(*buff++) << 8;
			safe_pgm_write(&word, addr + i, sizeof(word));
		}
	}
	else {
		/* posix EIO error */
		gdb_send_reply("E05");
		return;
	}
	gdb_send_reply("OK");
}

static bool_t gdb_insert_breakpoint(uint16_t rom_addr)
{
	uint16_t trap_opcode = TRAP_OPCODE;
	struct gdb_break *breakp = NULL;

	if (gdb_ctx->breaks_cnt == MAX_BREAKS)
		return FALSE;
	gdb_ctx->breaks_cnt++;

	for (uint8_t i = 0; i < ARRAY_SIZE(gdb_ctx->breaks); ++i) {
		if (!gdb_ctx->breaks[i].addr) {
			breakp = &gdb_ctx->breaks[i];
			break;
		}
	}
	/* we are sure breakp is not NULL */

	breakp->addr = rom_addr;
	breakp->opcode = safe_pgm_read_word((uint32_t)rom_addr << 1);
	safe_pgm_write(&trap_opcode, breakp->addr, sizeof(trap_opcode));
	return TRUE;
}

static void gdb_remove_breakpoint_ptr(struct gdb_break *breakp)
{
	safe_pgm_write(&breakp->opcode, breakp->addr, sizeof(breakp->opcode));
	breakp->addr = 0;
	gdb_ctx->breaks_cnt--;
}

static void gdb_remove_breakpoint(uint16_t rom_addr)
{
	struct gdb_break *breakp = gdb_find_break(rom_addr);
	gdb_remove_breakpoint_ptr(breakp);
}

static void gdb_insert_breakpoints_on_next_pc(uint16_t pc)
{
	uint16_t opcode;

	opcode = safe_pgm_read_word((uint32_t)pc << 1);

	/* TODO: need to handle devices with 22-bit PC */
	if ((opcode & CALL_MASK) == CALL_OPCODE ||
		(opcode & JMP_MASK) == JMP_OPCODE)
		gdb_insert_breakpoint(safe_pgm_read_word(((uint32_t)pc + 1) << 1));
	else if (opcode == ICALL_OPCODE || opcode == IJMP_OPCODE ||
			 opcode == EICALL_OPCODE || opcode == EIJMP_OPCODE)
		/* TODO: we do not handle EIND for EICALL/EIJMP opcode */
		gdb_insert_breakpoint((gdb_ctx->regs->r31 << 8) | gdb_ctx->regs->r30);
	else if ((opcode & RCALL_MASK) == RCALL_OPCODE ||
			 (opcode & RJMP_MASK) == RJMP_OPCODE)
		gdb_insert_breakpoint((opcode & REL_K_MASK) >> REL_K_SHIFT);
	else if ((opcode & RETn_MASK) == RETn_OPCODE)
		/* Return address will be upper on the stack */
		gdb_insert_breakpoint((*(&gdb_ctx->regs->pc_h + 2) << 8) |
							   *(&gdb_ctx->regs->pc_l + 2));
	else if ((opcode & CPSE_MASK) == CPSE_OPCODE ||
			 (opcode & SBRn_MASK) == SBRn_OPCODE ||
			 (opcode & SBIn_MASK) == SBIn_OPCODE) {
		/* These opcodes can jump to pc + 1, + 2 or + 3.
		   To avoid additional logic we simply set breaks on all of them */
		gdb_insert_breakpoint(pc + 1);
		gdb_insert_breakpoint(pc + 2);
		gdb_insert_breakpoint(pc + 3);
	}
	else if ((opcode & BRCH_MASK) == BRCH_OPCODE) {
		/* These opcodes can jump to pc + 1, + k + 1.
		   To avoid additional logic we simply set breaks on all of them */
		int8_t k = (opcode & BRCH_K_MASK) >> BRCH_K_SHIFT;
		/* k is 7-bits value and can be negative, so stretch 7 sign bit
		   over other bits */
		if (k & 0x40)
			k |= 0x80;
		gdb_insert_breakpoint(pc + 1);
		gdb_insert_breakpoint(pc + k + 1);
	}
	/* 32-bit opcode, advance 2 words */
	else if ((opcode & LDS_MASK) == LDS_OPCODE ||
			 (opcode & STS_MASK) == STS_OPCODE)
		gdb_insert_breakpoint(pc + 2);
	/* 16-bit opcode, advance 1 word */
	else
		gdb_insert_breakpoint(pc + 1);
}

static struct gdb_break *gdb_find_break(uint16_t rom_addr)
{
	for (uint8_t i = 0; i < ARRAY_SIZE(gdb_ctx->breaks); ++i)
		if (gdb_ctx->breaks[i].addr == rom_addr)
			return &gdb_ctx->breaks[i];
	return NULL;
}

static void gdb_insert_remove_breakpoint(const uint8_t *buff)
{
	uint32_t rom_addr_b, sz;
	uint8_t len;

	/* skip 'z0,' */
	len = parse_hex(buff + 3, &rom_addr_b);
	/* skip 'z0,xxx,' */
	parse_hex(buff + 3 + len + 1, &sz);

	/* get break type */
	switch (buff[1]) {
	case '0': /* software breakpoint */
		if (buff[0] == 'Z')
			gdb_insert_breakpoint(rom_addr_b >> 1);
		else
			gdb_remove_breakpoint(rom_addr_b >> 1);

		gdb_send_reply("OK");

		break;
	default:
		/* we do not support other breakpoints, only software */
		gdb_send_reply("");
		break;
	}
}

static void gdb_do_stepi(void)
{
	/* gdb guarantees that there will be no any breakpoints
	   already set on this stepi call, so do not bother about
	   overlapping breaks. Actually, I did not see this statement
	   in any gdb docs, but it behaves so (I saw traces). */

	gdb_insert_breakpoints_on_next_pc(gdb_ctx->pc);
	gdb_ctx->in_stepi = TRUE;
}

static bool_t gdb_parse_packet(const uint8_t *buff)
{
	switch (*buff) {
	case '?':               /* last signal */
		gdb_send_reply("S05"); /* signal # 5 is SIGTRAP */
		break;
	case 'H':               /* Set thread, always OK */
		gdb_send_reply("OK");
		break;
	case 'T':               /* Is thread alive, always OK */
		gdb_send_reply("OK");
		break;
	case 'g':               /* read registers */
		gdb_read_registers();
		break;
	case 'G':               /* write registers */
		gdb_write_registers(buff + 1);
		break;
	case 'p':               /* read a single register */
		gdb_read_register(buff + 1);
		break;
	case 'P':               /* write single register */
		gdb_write_register(buff + 1);
		break;
	case 'm':               /* read memory */
		gdb_read_memory(buff + 1);
		break;
	case 'M':               /* write memory */
		gdb_write_memory(buff + 1);
		break;
	case 'D':               /* detach the debugger */
	case 'k':               /* kill request */
		gdb_send_reply("OK");
		return FALSE;
	case 'c':               /* continue */
		return FALSE;
	case 'C':               /* continue with signal */
	case 'S':               /* step with signal */
		gdb_send_reply(""); /* not supported */
		break;
	case 's':               /* step */
		gdb_do_stepi();
		return FALSE;
	case 'z':               /* remove break/watch point */
	case 'Z':               /* insert break/watch point */
		gdb_insert_remove_breakpoint(gdb_ctx->buff);
		break;
	case 'q':               /* query requests */
		if(memcmp_PF(buff, (uintptr_t)PSTR("qSupported"), 10) == 0) {
			/* plain send, from ram */
			gdb_send_buff(gdb_pkt_sz_desc, 0, gdb_pkt_sz_desc_len, TRUE);
		} else if(memcmp_PF(buff, (uintptr_t)PSTR("qXfer:features:read:target.xml:"), 31) == 0) {
			/* GDB XML target descriptions, since GDB 6.7 (2007-10-10)
			   see http://sources.redhat.com/gdb/current/onlinedocs/gdb/Target-Descriptions.html */
			uint32_t len, off, sz;
			uint8_t b;
			struct buff_vec vec[] = { {.buff = &b, .off = 0,
									   .sz = 1, .in_pgm = FALSE},
									  {.buff = gdb_target_desc, .off = 0,
									   .sz = 0, .in_pgm = TRUE} };
			len = parse_hex(buff + 31, &off);
			parse_hex(buff + 31 + len + 1, &sz);
			if (off + sz > gdb_target_desc_len) {
				sz = gdb_target_desc_len - off;
				/* send is fully completed */
				b = 'l';
			} else
				/* will continue send with some offset */
				b = 'm';
			/* set actual off and sz */
			vec[1].off = off;
			vec[1].sz = sz;
			/* send vec */
			gdb_send_buff_vec(vec, ARRAY_SIZE(vec));
		} else if(memcmp_PF(gdb_ctx->buff, (uintptr_t)PSTR("qC"), 2) == 0)
			/* current thread is always 1 */
			gdb_send_reply("QC01");
		else if(memcmp_PF(gdb_ctx->buff, (uintptr_t)PSTR("qfThreadInfo"), 12) == 0)
			/* always 1 thread*/
			gdb_send_reply("m1");
		else if(memcmp_PF(gdb_ctx->buff, (uintptr_t)PSTR("qsThreadInfo"), 12) == 0)
			/* send end of list */
			gdb_send_reply("l");
		else
			gdb_send_reply("");  /* not supported */

		break;
	default:
		gdb_send_reply("");  /* not supported */
		break;
	}

	return TRUE;
}

static void gdb_trap(void)
{
	uint8_t checksum, pkt_checksum;
	uint8_t b;

	while (1) {
		b = gdb_read_byte();

		switch(b) {
		case '$':
			/* Read everything to buffer */
			gdb_ctx->buff_sz = 0;
			for (pkt_checksum = 0, b = gdb_read_byte();
				 b != '#'; b = gdb_read_byte()) {
				gdb_ctx->buff[gdb_ctx->buff_sz++] = b;
				pkt_checksum += b;
			}
			gdb_ctx->buff[gdb_ctx->buff_sz] = 0;

			checksum  = hex2nib(gdb_read_byte()) << 4;
			checksum |= hex2nib(gdb_read_byte());

			/* send nack in case of wrong checksum  */
			if (pkt_checksum != checksum) {
				gdb_send_byte('-');
				continue;
			}

			/* ack */
			gdb_send_byte('+');

			/* parse already read buffer */
			if (gdb_parse_packet(gdb_ctx->buff))
				continue;

			/* leave the trap, continue execution */
			return;

		case '-':  /* NACK, repeat previous reply */
			gdb_send_buff(gdb_ctx->buff, 0, gdb_ctx->buff_sz, FALSE);
			break;
		case '+':  /* ACK, great */
			break;
		case 0x03:
			/* user interrupt by Ctrl-C, send current state and
			   continue reading */
			gdb_send_state(GDB_SIGINT);
			break;
		default:
			gdb_send_reply(""); /* not supported */
			break;
		}
	}
}

/******************************************************************************/
