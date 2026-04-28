/*******************************************************************************
* File Name:   main.c
*
*
* Autor: Matthias Kleinlercher
* 
* HTL Anichstraße, Elektrotechnik und Mechatronik, 5AHET, 2025/26
*
*******************************************************************************

* This project is for testing the GateDrivers of U, V, W of PowerBoard2. U has a duty cycle of 25%, V of 50% and W of 75%.

*******************************************************************************/


/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cy_pdl.h"
#include "cy_tcpwm_pwm.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cycfg_peripherals.h"
#include "mtb_hal.h"
#include "cy_cordic.h"
#include "cy_device.h"
#include "cy_dma.h"
#include "cy_hppass_ac.h"
#include "cy_hppass_sar.h"
#include "cy_sysclk.h"
#include "cy_sysint.h"
#include "cy_syslib.h"
#include "cy_tcpwm.h"
#include "cy_tcpwm_counter.h"
#include "cy_tcpwm_pwm.h"
#include "cy_utils.h"
#include "cycfg_clocks.h"
#include "cycfg_peripherals.h"
#include "cycfg_pins.h"
#include "cyip_flashc.h"
#include "gpio_psc3_e_lqfp_80.h"
#include "mtb_hal.h"
#include "cybsp.h"
#include "psc3m5fds2afq1_s.h"
#include <stdint.h>
#include <stdio.h>
#include <cy_retarget_io.h>
#include <math.h>
#include <string.h>
#include "cy_gpio.h"
#include "cy_hppass.h"
#include "cy_cordic.h"
#include "cy_pdl.h"
#include <stdbool.h>

/*******************************************************************************
* Macros
*******************************************************************************/

/*******************************************************************************
* Global Variables
*******************************************************************************/

cy_stc_sysint_t fifo_isr_cfg             	   = { .intrSrc = pass_interrupt_fifos_IRQn, .intrPriority = 2U };

volatile bool timer_interrupt_flag = false;
bool Running = true;

/* Variable for storing character read from terminal */
uint8_t uart_read_value;
uint32_t Period = 0;
uint32_t idx = 0;
int16_t adc_sar_result[16] = {0};
uint32_t compareU = 0, compareV = 0, compareW = 0;

/* For the Retarget -IO (Debug UART) usage */
static cy_stc_scb_uart_context_t    DEBUG_UART_context;           /** UART context */
static mtb_hal_uart_t               DEBUG_UART_hal_obj;           /** Debug UART HAL object  */

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void timer_init(void);
/*******************************************************************************
* Function Definitions
*******************************************************************************/

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function. It sets up a timer to trigger a periodic interrupt.
* The main while loop checks for the status of a flag set by the interrupt and
* toggles an LED at 1Hz to create an LED blinky. Will be achieving the 1Hz Blink
* rate. The while loop also checks whether the 'Enter' key was pressed and
* stops/restarts LED blinking.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

#if defined (CY_DEVICE_SECURE)
    cyhal_wdt_t wdt_obj;

    /* Clear watchdog timer so that it doesn't trigger a reset */
    result = cyhal_wdt_init(&wdt_obj, cyhal_wdt_get_max_timeout_ms());
    CY_ASSERT(CY_RSLT_SUCCESS == result);
    cyhal_wdt_free(&wdt_obj);
#endif

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Debug UART init */
    result = (cy_rslt_t)Cy_SCB_UART_Init(DEBUG_UART_HW, &DEBUG_UART_config, &DEBUG_UART_context);

    /* UART init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    Cy_SCB_UART_Enable(DEBUG_UART_HW);

    /* Setup the HAL UART */
    result = mtb_hal_uart_setup(&DEBUG_UART_hal_obj, &DEBUG_UART_hal_config, &DEBUG_UART_context, NULL);

    /* HAL UART init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    result = cy_retarget_io_init(&DEBUG_UART_hal_obj);

	/* PWM U/V/W */
	if (result != Cy_TCPWM_PWM_Init(TCPWM_COUNTER_HW, TCPWM_COUNTER_NUM, &PWM_Counter_U_config)) { CY_ASSERT(0); }
    Cy_TCPWM_PWM_Enable(TCPWM_COUNTER_HW, TCPWM_COUNTER_NUM);
	
    if (result != Cy_TCPWM_PWM_Init(PWM_Counter_U_HW, PWM_Counter_U_NUM, &PWM_Counter_U_config)) { CY_ASSERT(0); }
    Cy_TCPWM_PWM_Enable(PWM_Counter_U_HW, PWM_Counter_U_NUM);

    if (result != Cy_TCPWM_PWM_Init(PWM_Counter_V_HW, PWM_Counter_V_NUM, &PWM_Counter_V_config)) { CY_ASSERT(0); }
    Cy_TCPWM_PWM_Enable(PWM_Counter_V_HW, PWM_Counter_V_NUM);

    if (result != Cy_TCPWM_PWM_Init(PWM_Counter_W_HW, PWM_Counter_W_NUM, &PWM_Counter_W_config)) { CY_ASSERT(0); }
    Cy_TCPWM_PWM_Enable(PWM_Counter_W_HW, PWM_Counter_W_NUM);
    
    if (result != Cy_TCPWM_PWM_Init(SevSw_Counter_HW, SevSw_Counter_NUM, &PWM_Counter_W_config)) { CY_ASSERT(0); }
    Cy_TCPWM_PWM_Enable(SevSw_Counter_HW, SevSw_Counter_NUM);
    
    uint32_t SevSwPeriod = Cy_TCPWM_PWM_GetPeriod0(SevSw_Counter_HW, SevSw_Counter_NUM);
    Cy_TCPWM_PWM_SetCompare0Val(SevSw_Counter_HW, SevSw_Counter_NUM, SevSwPeriod);

	Period =  4800;
	Cy_TCPWM_PWM_SetPeriod0(PWM_Counter_U_HW, PWM_Counter_U_NUM, Period);
	Cy_TCPWM_PWM_SetPeriod0(PWM_Counter_V_HW, PWM_Counter_V_NUM, Period);
	Cy_TCPWM_PWM_SetPeriod0(PWM_Counter_W_HW, PWM_Counter_W_NUM, Period);
	
	compareU = 1200;
	compareV = 2400;
	compareW = 3600;
	
	Cy_TCPWM_PWM_SetCompare0Val(PWM_Counter_U_HW, PWM_Counter_U_NUM, compareU);
	Cy_TCPWM_PWM_SetCompare0Val(PWM_Counter_V_HW, PWM_Counter_V_NUM, compareV);
	Cy_TCPWM_PWM_SetCompare0Val(PWM_Counter_W_HW, PWM_Counter_W_NUM, compareW);
	
	Cy_TCPWM_PWM_SetCompare0BufVal(PWM_Counter_U_HW, PWM_Counter_U_NUM, compareU);
	Cy_TCPWM_PWM_SetCompare0BufVal(PWM_Counter_V_HW, PWM_Counter_V_NUM, compareV);
	Cy_TCPWM_PWM_SetCompare0BufVal(PWM_Counter_W_HW, PWM_Counter_W_NUM, compareW);

	Cy_TCPWM_TriggerCaptureOrSwap_Single(PWM_Counter_U_HW, PWM_Counter_U_NUM);
	Cy_TCPWM_TriggerCaptureOrSwap_Single(PWM_Counter_V_HW, PWM_Counter_V_NUM);
    Cy_TCPWM_TriggerCaptureOrSwap_Single(PWM_Counter_W_HW, PWM_Counter_W_NUM);
	
	/* HPPASS/SAR */
    if(CY_HPPASS_SUCCESS != Cy_HPPASS_Init(&pass_0_config)) { CY_ASSERT(0); }
    Cy_HPPASS_AC_Start(0U, 100U);
    
	
	Cy_GPIO_Pin_Init(SW1_PORT, SW1_NUM, &SW1_config);

   	/* HAL retarget_io init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize timer to toggle the LED */
    timer_init();
	
	Cy_TCPWM_TriggerStart_Single(PWM_Counter_U_HW, PWM_Counter_U_NUM);
	Cy_TCPWM_TriggerStart_Single(PWM_Counter_V_HW, PWM_Counter_V_NUM);
	Cy_TCPWM_TriggerStart_Single(PWM_Counter_W_HW, PWM_Counter_W_NUM);
	Cy_TCPWM_TriggerStart_Single(SevSw_Counter_HW, SevSw_Counter_NUM);
	
    for (;;)
    {
	     if(true)
	     {
			
			// cyclic code
	
		 }
	}
}

/*******************************************************************************
 * Function Name: timer_init
 ********************************************************************************
 * Summary:
 * This function creates and configures a Timer object. The timer ticks
 * continuously and produces a periodic interrupt on every terminal count
 * event. The period is defined by the 'period' and 'compare_value' of the
 * timer configuration structure 'led_blink_timer_cfg'. Without any changes,
 * this application is designed to produce an interrupt every 1 second.
 *
 * Parameters:
 *  none
 *
 *******************************************************************************/
void timer_init(void)
{
     /* Enable interrupts */
     __enable_irq();
     
     /* Start the counter */
     Cy_TCPWM_TriggerStart_Single(TCPWM_COUNTER_HW, TCPWM_COUNTER_NUM);
}


