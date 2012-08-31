/******************************************************************************
 * Lightweight GDB server implementation for AVR MCU
 ******************************************************************************/
#include <avr/interrupt.h>
#include <avr/boot.h>
#include <avr/pgmspace.h>
#include <stddef.h>

#include "gdb.h"

#define ROUNDUP(x, s) (((x) + (s) - 1) & ~((s) - 1))
#define ROUNDDOWN(x, s) ((x) & ~((s) - 1))
#define ARRAY_SIZE(arr) (sizeof(arr)/sizeof((arr)[0]))

#if (SPM_PAGESIZE & (SPM_PAGESIZE - 1))
#error SPM_PAGESIZE is not power of two! Impossible!
#endif

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

struct gdb_context *gdb_ctx;
static void gdb_trap();

/******************************************************************************/

__attribute__((naked,noinline))
static void gdb_break_handler()
{
	GDB_SAVE_CONTEXT();
	gdb_ctx->int_reason = gdb_breakpoint;
	gdb_trap();
	GDB_RESTORE_CONTEXT();
	asm volatile ("ret \n\t");
}

//ISR(USART_RXC_vect, ISR_NAKED)
ISR(TIMER0_COMP_vect, ISR_NAKED)
{
	GDB_SAVE_CONTEXT();
	gdb_ctx->int_reason = gdb_user_interrupt;
	gdb_trap();
	GDB_RESTORE_CONTEXT();
	asm volatile ("reti \n\t");
}

/******************************************************************************/

void gdb_init(struct gdb_context *ctx)
{
	gdb_ctx = ctx;
	gdb_ctx->stack = NULL;
	gdb_ctx->int_reason = gdb_running;
	gdb_ctx->breaks_cnt = 0;

	/* Create 16-bit CALL instruction with address
	   of break handler */
	uintptr_t addr = (uintptr_t)&gdb_break_handler;
	for (uint8_t i = 0; i < ARRAY_SIZE(gdb_ctx->breaks); ++i) {
		uint8_t *inst = gdb_ctx->breaks[i].inst;
		inst[0] = 0x0e;
		inst[1] = 0x94;
		inst[2] = addr & 0xff;
		inst[3] = (addr >> 8) & 0xff;
	}

	//start uart
}

/* rom_addr is address in words, i.e. multiple of two
   NOTE: interrupts must be disabled before call of this func */
__attribute__ ((section(".nrww"),noinline))
static void __safe_pgm_write(void *ram_addr, void *rom_addr,
							 uint16_t sz)
{
	uintptr_t addr;
	uint16_t *ram = (uint16_t*)ram_addr;

	/* Sz must be valid and multiple of two */
	if (!sz || sz & 1)
		return;

	/* Avoid conflicts with EEPROM */
	eeprom_busy_wait();

	/* in bytes */
	addr = (uintptr_t)rom_addr * 2;

	for (uintptr_t page = ROUNDDOWN(addr, SPM_PAGESIZE),
		 end_page = ROUNDUP(addr + sz, SPM_PAGESIZE),
		 off = addr % SPM_PAGESIZE;
		 page < end_page;
		 page += SPM_PAGESIZE, off = 0) {

		/* Fill temporary page */
		for (uintptr_t page_off = 0;
			 page_off < SPM_PAGESIZE;
			 page_off += 2) {
			/* Fill with word from ram */
			if (page_off == off) {
				boot_page_fill(page + off,  *ram);
				if (sz -= 2) {
					off += 2;
					ram += 1;
				}
			}
			/* Fill with word from flash */
			else
				boot_page_fill(page + page_off,
							   pgm_read_word(page + page_off));
		}

		/* Erase page and wait until done. */
		boot_page_erase(page);
		boot_spm_busy_wait();

		/* Write page and wait until done. */
		boot_page_write(page);
		boot_spm_busy_wait();
	}
}

/* rom_addr in words */
static void gdb_rom_ram_swap_dword(void* rom_addr, void* ram_addr)
{
	uint32_t rom_dword = pgm_read_dword(rom_addr);
	uint32_t ram_dword = *(uint32_t*)ram_addr;
	if (rom_dword == ram_dword)
		return;
	*(uint32_t*)ram_addr = rom_dword;
	__safe_pgm_write(&ram_dword, rom_addr, sizeof(ram_dword));
}

static void gdb_trap()
{
	//XXX
	uint32_t v;
	gdb_rom_ram_swap_dword(0x00, &v);
#if 0
	uint8_t  c;

	while (1) {
		c = gdb_read_byte();

		switch(c) {
		case '$':
			/* continue reading bytes */
			continue;
		case '-':
			gdb_send_reply(gdb_last_reply(NULL));
			break;
		case '+':
			break;
		case 0x03: /* user interrupt by Ctrl-C */
			break;

			/*
			 * $ commands
			 */

		case '?':               /* last signal */
			gdb_send_reply("S05"); /* signal # 5 is SIGTRAP */
			break;
		case 'H':               /* Set thread */
			gdb_select_thread();
			break;
		case 'T':               /* Is thread alive */
			gdb_is_thread_alive();
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
			return;
		case 'c':               /* continue */
			return;
		case 'C':               /* continue with signal */
		case 'S':               /* step with signal */
			gdb_send_reply(""); /* not supported */
			break;
		case 's':               /* step */
			/* remove previous break
			   set break on next instruction */
			return;
		case 'z':               /* remove break/watch point */
		case 'Z':               /* insert break/watch point */
			gdb_break_point(pkt);
			break;
		case 'q':               /* query requests */
			pkt--;
			if(memcmp(pkt, "qSupported", 10) == 0) {
				gdb_send_reply("PacketSize=800;qXfer:features:read+");
				return;
			} else if(memcmp(pkt, "qXfer:features:read:target.xml:", 31) == 0) {
				/* GDB XML target descriptions, since GDB 6.7 (2007-10-10)
				 * see http://sources.redhat.com/gdb/current/onlinedocs/gdb/Target-Descriptions.html */
				gdb_send_reply("l"
							   "<?xml version=\"1.0\"?>\n"
							   "<!DOCTYPE target SYSTEM \"gdb-target.dtd\">\n"
							   "<target version=\"1.0\">\n"
							   "    <architecture>avr</architecture>\n"
							   "</target>\n");
				return GDB_RET_OK;
			} else if(strcmp(pkt, "qC") == 0) {
				int thread_id = core->stack->m_ThreadList.GetCurrentThreadForGDB();
				if (global_debug_on)
					fprintf(stderr, "gdb  get current thread: %d\n", thread_id);
				char reply[100];
				snprintf( reply, sizeof(reply), "QC%02x", thread_id);
				gdb_send_reply( reply );
				return GDB_RET_OK;
			} else if(strcmp(pkt, "qfThreadInfo") == 0) {
				gdb_get_thread_list(pkt);
				return GDB_RET_OK;
			} else if(strcmp(pkt, "qsThreadInfo") == 0) {
				gdb_send_reply(  "l" );  /* note lowercase "L" */
				return GDB_RET_OK;
			}

			gdb_send_reply("");  /* not supported */
			break;
		default:
			gdb_send_reply(""); /* not supported */
		}

	default:
		gdb_send_reply("-"); /* nak */
		break;
	}
#endif
}

/******************************************************************************/
