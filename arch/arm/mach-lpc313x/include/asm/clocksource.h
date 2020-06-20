/* LPC313x-specific clocksource additions */

#ifndef _ASM_LPC313X_CLOCKSOURCE_H
#define _ASM_LPC313X_CLOCKSOURCE_H

#ifdef CONFIG_ARCH_LPC313X

struct lpc313x_timer;
struct arch_clocksource_data {
	struct lpc313x_timer *timer;
};

#endif

#endif /* _ASM_LPC313X_CLOCKSOURCE_H */
