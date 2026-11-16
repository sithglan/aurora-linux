// SPDX-License-Identifier: GPL-2.0-only
/*
 * Low-level idle sequences
 */

#include <linux/cpu.h>
#include <linux/init.h>
#include <linux/irqflags.h>
#include <linux/printk.h>

#include <asm/apple-idle.h>
#include <asm/barrier.h>
#include <asm/cpuidle.h>
#include <asm/cpufeature.h>
#include <asm/cputype.h>
#include <asm/fpsimd.h>
#include <asm/sysreg.h>

enum {
    ARM64_IDLE_WFI,
    ARM64_IDLE_YIELD,
    ARM64_IDLE_NOP,
} idle = ARM64_IDLE_WFI;

static int __init setup_idle(char *arg)
{
	if (!arg)
		return -1;
	else if (!strcmp(arg, "wfi"))
		idle = ARM64_IDLE_WFI;
	else if (!strcmp(arg, "yield"))
		idle = ARM64_IDLE_YIELD;
	else if (!strcmp(arg, "nop"))
		idle = ARM64_IDLE_NOP;
	else
		return -1;

	return 0;
}
early_param("idle", setup_idle);

#ifdef CONFIG_ARCH_APPLE
/*
 * Preserve general-purpose and FP/SIMD state across WFI on M4 and later
 * Apple cores.
 *
 * The decision has to be made before the boot CPU first idles, which can
 * happen as soon as rest_init() starts its idle thread, long before any
 * initcall runs: on these SoCs the boot CPU loses its registers too. It is
 * made from smp_prepare_boot_cpu(), where static keys cannot be flipped
 * yet, so it is a plain flag.
 */
static bool apple_wfi_loses_regs __ro_after_init;
static bool apple_keep_idle_nop __initdata;

/*
 * The boot loader appends idle=nop after the user's arguments, so it cannot
 * be overridden on the command line. This flag keeps it anyway.
 */
static int __init setup_apple_keep_idle_nop(char *arg)
{
	apple_keep_idle_nop = true;
	return 0;
}
early_param("apple_keep_idle_nop", setup_apple_keep_idle_nop);

void __init apple_wfi_quirk_init(void)
{
	u32 model = read_cpuid_id() & MIDR_CPU_MODEL_MASK;

	if (midr_is_apple_m4_or_later(read_cpuid_id())) {
		apple_wfi_loses_regs = true;
		pr_info("Apple: preserving registers across WFI\n");
	}
	/*
	 * aura (J813) and m1n1 (T6050) pass idle=nop on M4 and later for
	 * kernels without this quirk. Spinning idle keeps every core busy
	 * (cpuidle's WFI states spin too): on T8142 the SoC sits near 90 C
	 * and throttles, on T6050 the system draws about 10 W more at idle.
	 * WFI is safe here.
	 */
	if (idle == ARM64_IDLE_NOP && !apple_keep_idle_nop &&
	    (model == MIDR_APPLE_T8142_E || model == MIDR_APPLE_T8142_P ||
	     model == MIDR_APPLE_T6050_064 || model == MIDR_APPLE_T6050_065)) {
		idle = ARM64_IDLE_WFI;
		pr_info("Apple: idling in WFI, ignoring idle=nop\n");
	}
}

/* TEST ONLY: capture CPU0 timer state without waking it with an IPI. */
static bool idle_debug;
static bool idle_debug_poll_cpu0;

void apple_idle_debug_poll(void)
{
	WRITE_ONCE(idle_debug_poll_cpu0, true);
}
static struct {
	u64 entries, exits, pct, vct, pctl, vctl, pcval, vcval, deep, hcr, cnthctl, vmgate;
} idle_debug_last;

void apple_idle_debug_dump(void)
{
	pr_info("TEST CPU0 idle entries=%llu exits=%llu pct=%#llx vct=%#llx pctl=%#llx vctl=%#llx pcval=%#llx vcval=%#llx deep=%#llx now=%#llx hcr=%#llx cnthctl=%#llx vmgate=%#llx\n",
		idle_debug_last.entries, idle_debug_last.exits,
		idle_debug_last.pct, idle_debug_last.vct,
		idle_debug_last.pctl, idle_debug_last.vctl,
		idle_debug_last.pcval, idle_debug_last.vcval,
		idle_debug_last.deep, read_sysreg(cntpct_el0),
		idle_debug_last.hcr, idle_debug_last.cnthctl, idle_debug_last.vmgate);
}

void apple_idle_debug_arm(void)
{
	idle_debug_last.entries = 0;
	idle_debug_last.exits = 0;
	WRITE_ONCE(idle_debug, true);
	pr_info("TEST resume CPU%u timer pct=%#llx vct=%#llx pctl=%#llx vctl=%#llx pcval=%#llx vcval=%#llx deep=%#llx\n",
		smp_processor_id(), read_sysreg(cntpct_el0), read_sysreg(cntvct_el0),
		read_sysreg(cntp_ctl_el0), read_sysreg(cntv_ctl_el0),
		read_sysreg(cntp_cval_el0), read_sysreg(cntv_cval_el0),
		read_sysreg_s(sys_reg(3, 5, 15, 6, 2)));
}

static void apple_idle_debug_enter(void)
{
	if (!READ_ONCE(idle_debug) || smp_processor_id())
		return;
	idle_debug_last.pct = read_sysreg(cntpct_el0);
	idle_debug_last.vct = read_sysreg(cntvct_el0);
	idle_debug_last.pctl = read_sysreg(cntp_ctl_el0);
	idle_debug_last.vctl = read_sysreg(cntv_ctl_el0);
	idle_debug_last.pcval = read_sysreg(cntp_cval_el0);
	idle_debug_last.vcval = read_sysreg(cntv_cval_el0);
	idle_debug_last.deep = read_sysreg_s(sys_reg(3, 5, 15, 6, 2));
	idle_debug_last.hcr = read_sysreg(hcr_el2);
	idle_debug_last.cnthctl = read_sysreg(cnthctl_el2);
	idle_debug_last.vmgate = read_sysreg_s(sys_reg(3, 5, 15, 1, 3));
	idle_debug_last.entries++;
}

static void apple_idle_debug_exit(void)
{
	if (READ_ONCE(idle_debug) && !smp_processor_id())
		idle_debug_last.exits++;
}

static __always_inline bool apple_wfi_needs_save(void)
{
	return unlikely(READ_ONCE(apple_wfi_loses_regs));
}
#else
static __always_inline bool apple_wfi_needs_save(void)
{
	return false;
}
#endif

/*
 *	cpu_do_idle()
 *
 *	Idle the processor (wait for interrupt).
 *
 *	If the CPU supports priority masking we must do additional work to
 *	ensure that interrupts are not masked at the PMR (because the core will
 *	not wake up if we block the wake up signal in the interrupt controller).
 */
void __cpuidle cpu_do_idle(void)
{
	struct arm_cpuidle_irq_context context;

	arm_cpuidle_save_irq_context(&context);

	if (READ_ONCE(idle_debug_poll_cpu0) && !smp_processor_id()) {
		apple_idle_debug_enter();
		apple_idle_debug_exit();
		arm_cpuidle_restore_irq_context(&context);
		return;
	}

	if (likely(idle == ARM64_IDLE_WFI)) {
		if (apple_wfi_needs_save()) {
			/*
			 * The FP/SIMD registers do not survive either: drop the
			 * lazy ownership so the next return to userspace reloads
			 * them, even when the same task resumes on this CPU.
			 */
			fpsimd_save_and_flush_cpu_state();
			apple_idle_debug_enter();
			apple_cpu_wfi();
			apple_idle_debug_exit();
		} else {
			dsb(sy);
			wfi();
		}
	} else if (idle == ARM64_IDLE_YIELD) {
		dsb(sy);
		asm volatile("yield" ::: "memory");
	}

	arm_cpuidle_restore_irq_context(&context);
}

/*
 * This is our default idle handler.
 */
void __cpuidle arch_cpu_idle(void)
{
	/*
	 * This should do all the clock switching and wait for interrupt
	 * tricks
	 */
	cpu_do_idle();
}
