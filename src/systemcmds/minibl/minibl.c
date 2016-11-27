/*
* =====================================================================================
*
*       Filename:  minibl.c
*
*    Description:  a mini bootloader that just read the initialization vector
*                  and call __start with the required arguments.
*
*        Version:  1.0
*        Created:  24/11/2016 21:10:33
*       Revision:  none
*       Compiler:  arm-none-eabi-gcc
*
*         Author:  Zakaria ElQotbi (zskdan), zakaria.elqotbi@derbsellicon.com
*        Company:  Derbsellicon
*
* =====================================================================================
*/

#include <stdint.h>

#define MMIO32(addr) (*(volatile uint32_t *)(addr))

#define APP_LOAD_ADDRESS 0x00400000
#define VTOR_ADDRESS     0xE000ED08
#define SCB_VTOR         MMIO32(VTOR_ADDRESS)

__EXPORT int minibl_main(int argc, char *argv[]);

static void do_jump(uint32_t stacktop, uint32_t entrypoint);

int minibl_main(int argc, char *argv[])
{
	const uint32_t *app_base = (const uint32_t *)APP_LOAD_ADDRESS;

	/*
	 * We refuse to program the first word of the app until the upload is marked
	 * complete by the host.  So if it's not 0xffffffff, we should try booting it.
	 */
	if (app_base[0] == 0xffffffff) {
		return -1;
	}

	/*
	 * The second word of the app is the entrypoint; it must point within the
	 * flash area (or we have a bad flash).
	 */
	if (app_base[1] < APP_LOAD_ADDRESS) {
		return -1;
	}

#if 0
	/* switch exception handlers to the application */
	SCB_VTOR = APP_LOAD_ADDRESS;
#endif

	/* extract the stack and entrypoint from the app vector table and go */
	do_jump(app_base[0], app_base[1]);

	return 0;
}

static void
do_jump(uint32_t stacktop, uint32_t entrypoint)
{
	asm volatile(
		"msr msp, %0	\n"
		"bx	%1	\n"
		: : "r"(stacktop), "r"(entrypoint) :);

	// just to keep noreturn happy
	for (;;) ;
}

