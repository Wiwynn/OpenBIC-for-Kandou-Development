#include "plat_sys.h"

#include "util_sys.h"
#include "hal_gpio.h"
#include "plat_gpio.h"

void pal_warm_reset_prepare()
{
	ME_enter_recovery();
}

void pal_cold_reset_prepare()
{
	ME_enter_recovery();
}

/* BMC reset */
void BMC_reset_handler()
{
	gpio_set(RST_BMC_R_N, GPIO_LOW);
	k_msleep(10);
	gpio_set(RST_BMC_R_N, GPIO_HIGH);
}

K_DELAYED_WORK_DEFINE(BMC_reset_work, BMC_reset_handler);
int pal_submit_bmc_cold_reset()
{
	k_work_schedule(&BMC_reset_work, K_MSEC(1000));
	return 0;
}
/* BMC reset */