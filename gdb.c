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

/* CALL, JMP, LDS, STS
   1111 1110 0000 1110 */
#define MASK_32_OPCODE 0xfe0e

/* CALL
   1001 010k kkkk 111k
   kkkk kkkk kkkk kkkk */
#define CALL_OPCODE    0x940e

/* JMP
   1001 010k kkkk 110k
   kkkk kkkk kkkk kkkk */
#define JMP_OPCODE     0x940c

/* LDS
   1001 000d dddd 0000
   kkkk kkkk kkkk kkkk */
#define LDS_OPCODE     0x9000

/* STS
   1001 001d dddd 0000
   kkkk kkkk kkkk kkkk */
#define STS_OPCODE     0x9200

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

static void gdb_trap();
static void gdb_remove_breakpoint(uint16_t rom_addr);
static void gdb_send_state(uint8_t signo);

/* Convert number 0-15 to hex */
#define nib2hex(i) (uint8_t)(i > 9 ? 'a' - 10 + i : '0' + i)

/* Convert a hexidecimal digit to a 4 bit nibble. */
static inline uint8_t hex2nib(uint8_t hex)
{
	if (hex >= 'A' && hex <= 'F')
		return 10 + (hex - 'A');
	else if (hex >= 'a' && hex <= 'f')
		return 10 + (hex - 'a');
	else if (hex >= '0' && hex <= '9')
		return hex - '0';

	/* unreachable line */
	return 0;
}

static inline bool_t is_32bit_opcode(uint16_t opcode)
{
	opcode &= MASK_32_OPCODE;
	return (opcode == CALL_OPCODE ||
			opcode == JMP_OPCODE  ||
			opcode == LDS_OPCODE  ||
			opcode == STS_OPCODE);
}

static inline struct gdb_break *gdb_find_break(uint16_t rom_addr)
{
	for (uint8_t i = 0; i < gdb_ctx->breaks_cnt; ++i)
		if (gdb_ctx->breaks[i].addr == rom_addr)
			return &gdb_ctx->breaks[i];
}

/******************************************************************************/

ISR(INT0_vect, ISR_NAKED)
{
	struct gdb_break *breakp;

	/* Context save must be the first */
	GDB_SAVE_CONTEXT();
	gdb_ctx->pc = (gdb_ctx->regs->ret_addr_h << 8) |
				  (gdb_ctx->regs->ret_addr_l);
	/* We should continue execution from the PC where
	   trap opcode was set, so decrement 1 word */
	gdb_ctx->pc -= 1;
	/* Replace return address with corrected PC */
	gdb_ctx->regs->ret_addr_h = (gdb_ctx->pc >> 8) & 0xff;
	gdb_ctx->regs->ret_addr_l = gdb_ctx->pc & 0xff;

	/* Set correct interrupt reason */
	if (gdb_ctx->in_stepi) {
		/* Remove previous stepi break */
		gdb_remove_breakpoint(gdb_ctx->pc);
		gdb_ctx->int_reason = gdb_stepi_breakpoint;
		gdb_ctx->in_stepi = FALSE;
	}
	else
		gdb_ctx->int_reason = gdb_orig_breakpoint;

	gdb_send_state(GDB_SIGTRAP);
	gdb_trap();
	GDB_RESTORE_CONTEXT();
	asm volatile ("reti \n\t");
}

ISR(USART_RXC_vect, ISR_NAKED)
{
	GDB_SAVE_CONTEXT();
	gdb_ctx->pc = (gdb_ctx->regs->ret_addr_h << 8) |
				  (gdb_ctx->regs->ret_addr_l);
	gdb_ctx->int_reason = gdb_user_interrupt;
	gdb_trap();
	GDB_RESTORE_CONTEXT();
	asm volatile ("reti \n\t");
}

/******************************************************************************/

void gdb_init(struct gdb_context *ctx)
{
	/* Init static ptrs to program mem */
	gdb_target_desc = PSTR(
		"<?xml version=\"1.0\"?>\n"
		"<!DOCTYPE target SYSTEM \"gdb-target.dtd\">\n"
		"<target version=\"1.0\">\n"
		"  <architecture>avr</architecture>\n"
		"</target>\n");
	gdb_target_desc_len = strlen_PF((uintptr_t)gdb_target_desc);

	/* NOTE: PacketSize=0xXX, i.e. MAX_BUFF must be in hex */
	gdb_pkt_sz_desc = PSTR(
		"PacketSize=" STR_VAL(MAX_BUFF) ";qXfer:features:read+");
	gdb_pkt_sz_desc_len = strlen_PF((uintptr_t)gdb_pkt_sz_desc);

	/* Init gdb context */
	gdb_ctx = ctx;
	gdb_ctx->stack = NULL;
	gdb_ctx->breaks_cnt = 0;
	gdb_ctx->buff_sz = 0;
	gdb_ctx->in_stepi = FALSE;

	/* Create 16-bit trap opcode for software interrupt */
	for (uint8_t i = 0; i < ARRAY_SIZE(gdb_ctx->breaks); ++i)
		gdb_ctx->breaks[i].opcode = TRAP_OPCODE;

	/* init, start int0 software interrupt */
	/* init, start uart */
}

static inline uint16_t safe_pgm_read_word(uint32_t rom_addr_b)
{
#ifdef pgm_read_word_far
	if (rom_addr_b >= (1l<<16))
		return pgm_read_word_far(rom_addr_b);
	else
#endif
		return pgm_read_word(rom_addr_b);
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
		boot_page_erase(page);
		boot_spm_busy_wait();

		/* Write page and wait until done. */
		boot_page_write(page);
		boot_spm_busy_wait();
	}
}

static void gdb_send_byte(uint8_t b)
{
	/* need to implement */
	UDR = b;
}


static uint8_t gdb_read_byte()
{
	/* todo */
	return UDR;
}

static void gdb_send_buff(const uint8_t *buff, uint8_t off,
						  uint8_t sz, bool_t in_hex, bool_t in_pgm)
{
	uint8_t sum = 0;
	uint8_t b;

	gdb_send_byte('$');

	for (uint8_t i = 0; i < sz; ++i) {
		if (in_pgm)
			/* NOTE: rom buffer must be located in < 64K address section */
			b = pgm_read_byte(&buff[i + off]);
		else
			b = buff[i + off];

		sum += b;

		if (in_hex) {
			gdb_send_byte(nib2hex((b >> 4) & 0xf));
			gdb_send_byte(nib2hex(b & 0xf));
		} else
			gdb_send_byte(b);
	}

	gdb_send_byte('#');
	gdb_send_byte(nib2hex((sum >> 4) & 0xf));
	gdb_send_byte(nib2hex(sum & 0xf));
}

static void gdb_send_reply(const char *reply)
{
	uint8_t len = strlen(reply);
	gdb_ctx->buff_sz = MIN(len, sizeof(gdb_ctx->buff));
	memcpy(gdb_ctx->buff, reply, gdb_ctx->buff_sz);
	gdb_send_buff(gdb_ctx->buff, 0, gdb_ctx->buff_sz, FALSE, FALSE);
}

static void gdb_send_state(uint8_t signo)
{
	uint32_t pc = (uint32_t)gdb_ctx->pc << 1;

	gdb_ctx->buff_sz = snprintf_P(gdb_ctx->buff, sizeof(gdb_ctx->buff),
								  PSTR("T%02x20:%02x;21:%02x%02x;22:%02x%02x"
									   "%02x%02x;thread:%d;"),
								  signo,
								  gdb_ctx->regs->sreg,
								  (uintptr_t)gdb_ctx->stack & 0xff,
								  ((uintptr_t)gdb_ctx->stack >> 8) & 0xff,
								  pc & 0xff,
								  (pc >> 8) & 0xff,
								  (pc >> 16) & 0xff,
								  (pc >> 24) & 0xff,
								  /* thread id, always 1 */
								  1);

	/* not in hex, send from ram */
	gdb_send_buff(gdb_ctx->buff, 0, gdb_ctx->buff_sz,
				  FALSE, FALSE);
}

static void gdb_read_registers()
{
}

static void gdb_write_registers()
{
}

static void gdb_read_register()
{
}

static void gdb_write_register()
{
}

static void gdb_read_memory()
{
}

static void gdb_write_memory()
{
}

static bool_t gdb_insert_breakpoint(uint16_t rom_addr)
{
	uint16_t trap_opcode;
	struct gdb_break *breakp;

	if (gdb_ctx->breaks_cnt == MAX_BREAKS)
		return FALSE;

	trap_opcode = TRAP_OPCODE;
	breakp = &gdb_ctx->breaks[gdb_ctx->breaks_cnt++];
	breakp->addr = rom_addr;
	breakp->opcode = safe_pgm_read_word((uint32_t)rom_addr << 1);
	safe_pgm_write(&trap_opcode, breakp->addr, sizeof(trap_opcode));
	return TRUE;
}

static void gdb_remove_breakpoint(uint16_t rom_addr)
{
	struct gdb_break *breakp = gdb_find_break(rom_addr);
	safe_pgm_write(&breakp->opcode, breakp->addr, sizeof(breakp->opcode));
	breakp->addr = 0;
	gdb_ctx->breaks_cnt--;
}

static uint8_t gdb_parse_hex(const uint8_t *buff, uint32_t *hex)
{
	(void)buff;
	(void)hex;
}

static void gdb_insert_remove_breakpoint(const uint8_t *buff)
{
	uint32_t rom_addr_b, sz;
	uint8_t len;

	/* skip 'z0,' */
	len = gdb_parse_hex(buff + 3, &rom_addr_b);
	/* skip 'z0,xxx,' */
	gdb_parse_hex(buff + 3 + len + 1, &sz);

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

static void gdb_do_stepi()
{
	struct gdb_break *breakp;
	uint16_t opcode;
	uint16_t next_pc;

	/* gdb guarantees that there will be no any breakpoints
	   already set on this stepi call, so do not bother about
	   overlapping breaks. Actually, I did not see this statement
	   in any gdb docs, but it behaves so (I saw traces). */

	opcode = safe_pgm_read_word((uint32_t)gdb_ctx->pc << 1);

	/* Get next PC */
	if (is_32bit_opcode(opcode))
		next_pc = gdb_ctx->pc + 2;
	else
		next_pc = gdb_ctx->pc + 1;

	/* We are sure here that breaks are removed and we will not
	   get any errors, so do not check anything */
	gdb_insert_breakpoint(next_pc);
	gdb_ctx->in_stepi = TRUE;
}

static inline bool_t gdb_parse_packet(const char *buff)
{
	switch (*buff) {
	case '?':               /* last signal */
		gdb_send_reply("S05"); /* signal # 5 is SIGTRAP */
		break;
	case 'H':               /* Set thread, not supported */
		gdb_send_reply("E00");
		break;
	case 'T':               /* Is thread alive, always OK */
		gdb_send_reply("OK");
		break;
	case 'g':               /* read registers */
		gdb_read_registers();
		break;
	case 'G':               /* write registers */
		gdb_write_registers();
		break;
	case 'p':               /* read a single register */
		gdb_read_register();
		break;
	case 'P':               /* write single register */
		gdb_write_register();
		break;
	case 'm':               /* read memory */
		gdb_read_memory();
		break;
	case 'M':               /* write memory */
		gdb_write_memory();
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
			gdb_send_buff(gdb_pkt_sz_desc, 0, gdb_pkt_sz_desc_len,
						  FALSE, TRUE);
		} else if(memcmp_PF(buff, (uintptr_t)PSTR("qXfer:features:read:target.xml:"), 31) == 0) {
			/* GDB XML target descriptions, since GDB 6.7 (2007-10-10)
			   see http://sources.redhat.com/gdb/current/onlinedocs/gdb/Target-Descriptions.html */
			uint8_t off = 0, sz = 0;
			if (off + sz > gdb_target_desc_len) {
				sz = gdb_target_desc_len - off;
				/* send is fully completed */
				gdb_send_byte('l');
			} else
				/* will continue send with some offset */
				gdb_send_byte('m');
			/* plain send, from ram */
			gdb_send_buff(gdb_target_desc, off, sz,
						  FALSE, TRUE);
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

static void gdb_trap()
{
	uint8_t checksum, pkt_checksum;
	uint8_t b;

	while (1) {
		b = gdb_read_byte();

		switch(b) {
		case '$':
			/* Read everything to buffer */
			for (pkt_checksum = 0, b = gdb_read_byte();
				 b != '#'; b = gdb_read_byte()) {
				gdb_ctx->buff[gdb_ctx->buff_sz++] = b;
				pkt_checksum += b;
			}

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
			gdb_send_buff(gdb_ctx->buff, 0, gdb_ctx->buff_sz,
						  FALSE, FALSE);
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
