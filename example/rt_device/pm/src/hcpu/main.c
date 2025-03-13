/**
  ******************************************************************************
  * @file   main.c
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2021 - 2021,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>
#include "data_service_subscriber.h"
#include "ancs_service.h"
#include "flashdb.h"
#include "drv_flash.h"

#define LOG_TAG "pm"
#include "log.h"


static void gpio_wakeup_handler(void *args)
{
    rt_kprintf("gpio_wakeup_handler!\n");
}
static void app_wakeup(void)
{
    uint8_t pin;
#if defined(SF32LB55X)
    HAL_PIN_Set(PAD_PA80, GPIO_A80, PIN_PULLUP, 1); //PA80 #WKUP_A3

    HAL_StatusTypeDef status = HAL_HPAON_EnableWakeupSrc(HPAON_WAKEUP_SRC_PIN3, AON_PIN_MODE_LOW); //Enable #WKUP_A3(PA80)

    rt_pin_attach_irq(80, PIN_IRQ_MODE_RISING_FALLING, (void *) gpio_wakeup_handler,
                      (void *)(rt_uint32_t) 80);
    rt_pin_irq_enable(80, 1);

#elif defined(SF32LB56X)
#define WAKE_KEY (96+32) //PB32 #WKUP_PIN0

    HAL_StatusTypeDef status = HAL_HPAON_EnableWakeupSrc(HPAON_WAKEUP_SRC_PIN0, AON_PIN_MODE_POS_EDGE); //Enable #WKUP_PIN0 (PB32)

    rt_pin_mode(WAKE_KEY, PIN_MODE_INPUT);
    rt_pin_attach_irq(WAKE_KEY, PIN_IRQ_MODE_RISING, (void *) gpio_wakeup_handler,
                      (void *)(rt_uint32_t) WAKE_KEY); //PB32 GPIO interrupt
    rt_pin_irq_enable(WAKE_KEY, 1);
#elif defined(SF32LB52X)

    int8_t wakeup_pin;
    HAL_PIN_Set(PAD_PA34, GPIO_A34, PIN_PULLDOWN, 1); //set PA34 to GPIO funtion

    HAL_HPAON_EnableWakeupSrc(HPAON_WAKEUP_SRC_PIN10, AON_PIN_MODE_POS_EDGE); //Enable #WKUP_PIN10 (PA34)

    rt_pin_mode(BSP_KEY1_PIN, PIN_MODE_INPUT);

    rt_pin_attach_irq(BSP_KEY1_PIN, PIN_IRQ_MODE_RISING, (void *) gpio_wakeup_handler,
                      (void *)(rt_uint32_t) BSP_KEY1_PIN); //PA34 GPIO interrupt
    rt_pin_irq_enable(BSP_KEY1_PIN, 1);

#elif defined(SF32LB58X)
#define WAKE_KEY (96+54) //PB54 #WKUP_PIN0

    HAL_PIN_Set(PAD_PB54, GPIO_B54, PIN_NOPULL, 0);
    HAL_StatusTypeDef status = HAL_HPAON_EnableWakeupSrc(HPAON_WAKEUP_SRC_PIN0, AON_PIN_MODE_HIGH); //enable #WKUP_PIN0 (PB54)

    rt_pin_mode(WAKE_KEY, PIN_MODE_INPUT);
    rt_pin_attach_irq(WAKE_KEY, PIN_IRQ_MODE_FALLING, (void *) gpio_wakeup_handler,
                      (void *)(rt_uint32_t) WAKE_KEY); //PB54 GPIO interrupt
    rt_pin_irq_enable(WAKE_KEY, 1);
    rt_kprintf("SF32LB58X AON CR1:0x%x,CR2:0x%x,WER:0x%x\n", hwp_hpsys_aon->CR1, hwp_hpsys_aon->CR2, hwp_hpsys_aon->WER);

#else

#endif

}

void rc10k_timeout_handler(void *parameter)
{
    HAL_RC_CAL_update_reference_cycle_on_48M(LXT_LP_CYCLE);
}
#if defined(SF32LB52X)||defined(SF32LB58X)
    HAL_RAM_RET_CODE_SECT(sleep,   int sleep(int argc, char **argv))
#else
    int sleep(int argc, char **argv)
#endif
{
    char i;
    if (argc > 1)
    {
        if (strcmp("standby", argv[1]) == 0)
        {
            rt_kprintf("sleep on\r\n");
            rt_pm_release(PM_SLEEP_MODE_IDLE);
        }
        else if (strcmp("off", argv[1]) == 0)
        {
            rt_kprintf("sleep off\r\n");
            rt_pm_request(PM_SLEEP_MODE_IDLE);
        }
        else if (strcmp("down", argv[1]) == 0)
        {
#if defined(SF32LB55X)
            rt_kprintf("SF32LB55X entry_hibernate\n");
            HAL_PIN_Set(PAD_PB48, GPIO_B48, PIN_NOPULL, 0);
            HAL_PMU_EnablePinWakeup(5, AON_PIN_MODE_NEG_EDGE); //PB48 #WKUP_PIN5
            rt_kprintf("SF32LB55X CR:0x%x,WER:0x%x\n", hwp_pmuc->CR, hwp_pmuc->WER);
            rt_hw_interrupt_disable();
            HAL_PMU_EnterHibernate();
            while (1);
#elif defined(SF32LB52X)
            rt_kprintf("SF32LB52X entry_hibernate\n");
            HAL_PMU_SelectWakeupPin(0, HAL_HPAON_QueryWakeupPin(hwp_gpio1, BSP_KEY1_PIN)); //select PA34 to wake_pin0
            HAL_PMU_EnablePinWakeup(0, AON_PIN_MODE_HIGH);  //enable wake_pin0
            hwp_pmuc->WKUP_CNT = 0x50005;    //31-16bit:config PIN1 wake CNT , 15-0bit:PIN0 wake CNT
            rt_kprintf("SF32LB52X CR:0x%x,WER:0x%x\n", hwp_pmuc->CR, hwp_pmuc->WER);
            rt_hw_interrupt_disable();
            HAL_PMU_ConfigPeriLdo(PMUC_PERI_LDO_EN_VDD33_LDO3_Pos, false, false);
            HAL_PMU_ConfigPeriLdo(PMUC_PERI_LDO_EN_VDD33_LDO2_Pos, false, false);
            HAL_PMU_ConfigPeriLdo(PMU_PERI_LDO_1V8, false, false);
            HAL_PMU_EnterHibernate();
            while (1);
#elif defined(SF32LB58X)
            rt_kprintf("SF32LB58X entry_hibernate\n");

            HAL_PIN_Set(PAD_PB54, GPIO_B54, PIN_PULLDOWN, 0);  //Set PB54(#WKUP_PIN0) to GPIO and pulldown
            HAL_PMU_SelectWakeupPin(0, HAL_HPAON_QueryWakeupPin(hwp_gpio2, 54)); //select PB54 to pin0
            HAL_PMU_EnablePinWakeup(0, AON_PIN_MODE_HIGH);  //enable wake_pin0

            rt_kprintf("SF32LB5XX CR:0x%x,WER:0x%x,WKUP_CNT:0x%x,PBR0R:0x%x\n", hwp_pmuc->CR, hwp_pmuc->WER, hwp_pmuc->WKUP_CNT, hwp_rtc->PBR0R);
            rt_hw_interrupt_disable();
            HAL_PBR0_FORCE1_DISABLE();
            //HAL_PIN_Set(PAD_PBR0,PBR_GPO, PIN_NOPULL, 0);
            HAL_PBR_ConfigMode(0, 0); //set PBR0 input
            //HAL_PBR_ConfigMode(0, 1); //set PBR0 output
            //HAL_PBR_WritePin(0, 0); //set PBR0 low
            rt_kprintf("SF32LB58X PBR0R:0x%x\n", hwp_rtc->PBR0R);
            HAL_PMU_EnterHibernate();
            while (1);
#else
            rt_kprintf("DO NOT support hibernate\n");
#endif
        }
        else
        {
            rt_kprintf("sleep cmd err\r\n");
        }
    }
    return 0;
}

MSH_CMD_EXPORT(sleep, forward sleep command); /* 导出到 msh 命令列表中 */

int main(void)
{
    //*(volatile uint32_t *)0x4004f000 = 1;
    MODIFY_REG(hwp_pmuc->LXT_CR, PMUC_LXT_CR_BM_Msk, MAKE_REG_VAL(0xF, PMUC_LXT_CR_BM_Msk, PMUC_LXT_CR_BM_Pos));   // Increase current

#ifdef BSP_USING_PM
    rt_pm_request(PM_SLEEP_MODE_IDLE);
    rt_kprintf("hcpu main!!!\n");
    rt_thread_delay(3000);
    rt_pm_release(PM_SLEEP_MODE_IDLE);
    app_wakeup();

    if (PM_HIBERNATE_BOOT == SystemPowerOnModeGet())
        rt_kprintf("boot from hibernate!!!\n");
#endif

    while (1)
    {
#if defined(LXT_DISABLE)
        if (HAL_PMU_LXT_DISABLED())
        {
            rt_thread_mdelay(5 * 60 * 1000); // 5 minutes
            drv_rtc_calculate_delta(0);
            rt_kprintf("rc 10k calibration!!!\n");
        }
        else
#endif
        {
            rt_thread_mdelay(5000);
            rt_kprintf("hcpu timer wakeup!!!\n");
        }
    }
    return RT_EOK;
}



/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

