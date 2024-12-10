/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the HPPASS SAR ADC conversion time
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cybsp.h"
#include "cy_pdl.h"
#include "mtb_hal.h"
#include "cy_retarget_io.h"


/*******************************************************************************
* Macros
********************************************************************************/
/* The definition of HPPASS AC startup timeout in microseconds.
 * HPPASS startup time contains AREF startup 40us, CSG startup about 15us and 
 * SAR ADC maximum self-calibration time 9ms (HPPASS input clock is 240MHz). To be
 * on the safe side, add to 10ms.
 */
#define HPPASS_AC_STARTUP_TIMEOUT           (10000U)

/* The number of SAR ADC channels in group 0 */
#define NUM_ADC_CHANNELS_OF_GROUP0          (16U)

/*******************************************************************************
* Global Variables
********************************************************************************/
/* For the Retarget-IO (Debug UART) usage */
static cy_stc_scb_uart_context_t  DEBUG_UART_context;   /* Debug UART context */
static mtb_hal_uart_t DEBUG_UART_hal_obj;               /* Debug UART HAL object */

/* The interrupt configuration structure for HPPASS SAR ADC group 0 done */
cy_stc_sysint_t adc_group0_done_intr_config =
{
    .intrSrc = pass_interrupt_sar_entry_done_0_IRQn,
    .intrPriority = 0U,
};

/* ADC group 0 done interrupt flag */
volatile bool adc_group0_done_int_flag = false;

/* The status of adc group 0 conversion */
volatile bool adc_group0_is_converting= false;

/* ADC group 0 conversion count */
uint32_t adc_group0_conversion_count = 0;

/*******************************************************************************
* Function Prototypes
********************************************************************************/
/* ADC group 0 done interrupt handler */
void adc_group0_done_intr_handler(void);

/* Check if the user button is pressed */
bool user_button_is_pressed(void);


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CPU.
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

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize the debug UART */
    result = Cy_SCB_UART_Init(DEBUG_UART_HW, &DEBUG_UART_config, &DEBUG_UART_context);
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

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(&DEBUG_UART_hal_obj);
    /* retarget-io init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize user timer using the config structure generated using device configurator*/
    if (CY_TCPWM_SUCCESS != Cy_TCPWM_Counter_Init(USER_TIMER_HW, USER_TIMER_NUM, &USER_TIMER_config))
    {
        CY_ASSERT(0);
    }
    /* Enable the initialized user timer */
    Cy_TCPWM_Counter_Enable(USER_TIMER_HW, USER_TIMER_NUM);

    /* Initialize user counter for count SAR ADC conversion time */
    if (CY_TCPWM_SUCCESS != Cy_TCPWM_Counter_Init(USER_COUNTER_HW, USER_COUNTER_NUM, &USER_COUNTER_config))
    {
        CY_ASSERT(0);
    }
    /* Enable the initialized user counter */
    Cy_TCPWM_Counter_Enable(USER_COUNTER_HW, USER_COUNTER_NUM);

    /* Get the frequency of user counter input clock */
    uint32_t user_counter_clk_hz = Cy_SysClk_PeriPclkGetFrequency((en_clk_dst_t)USER_COUNTER_DIV_GRP_NUM, 
                                                        CY_SYSCLK_DIV_16_BIT, USER_COUNTER_DIV_NUM);
    float32_t user_counter_clk_mhz = (float32_t)user_counter_clk_hz / 1000000;

    /* Configure ADC group 0 done interrupt */
    Cy_HPPASS_SAR_Result_SetInterruptMask(CY_HPPASS_INTR_SAR_RESULT_GROUP_0);
    Cy_SysInt_Init(&adc_group0_done_intr_config, adc_group0_done_intr_handler);
    NVIC_EnableIRQ(adc_group0_done_intr_config.intrSrc);

    /* Start the HPPASS autonomous controller (AC) from state 0 */
    if(CY_HPPASS_SUCCESS != Cy_HPPASS_AC_Start(0U, HPPASS_AC_STARTUP_TIMEOUT))
    {
        CY_ASSERT(0);
    }

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");
    printf("********************************************************************************\r\n");
    printf("HPPASS: SAR ADC conversion time example\r\n");
    printf("********************************************************************************\r\n");
    printf("Press user switch (SW2) to start or stop SAR ADC conversion\r\n");

    /* Enable global interrupts */
    __enable_irq();

    for (;;)
    {
        /* Check if 'SW2' key was pressed */
        if(user_button_is_pressed())
        {
            if(!adc_group0_is_converting)
            {
                printf("\r\nStart SAR ADC group 0 conversion\r\n");
                Cy_TCPWM_TriggerStart_Single(USER_TIMER_HW, USER_TIMER_NUM);
                adc_group0_is_converting = true;
            }
            else
            {
                Cy_TCPWM_TriggerStopOrKill_Single(USER_TIMER_HW, USER_TIMER_NUM);
                printf("Stop SAR ADC group 0 conversion\r\n");
                adc_group0_is_converting = false;
            }
        }

        /* Check whether the ADC conversion is complete */
        if(adc_group0_done_int_flag)
        {
            adc_group0_done_int_flag = false;
            float32_t conversion_time = (float32_t)adc_group0_conversion_count / (float32_t)user_counter_clk_mhz;
            printf("SAR ADC %d channels conversion time: %.03fuS\r\n", NUM_ADC_CHANNELS_OF_GROUP0, conversion_time);
            /* Set the counter to 0 */
            Cy_TCPWM_Counter_SetCounter(USER_COUNTER_HW, USER_COUNTER_NUM, 0);
        }
    }
}

/*******************************************************************************
* Function Name: adc_group0_done_intr_handler
********************************************************************************
* Summary:
* This function is the ADC group 0 done interrupt handler
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void adc_group0_done_intr_handler(void)
{
    /* Get the count of ADC group 0 conversion */
    adc_group0_conversion_count = Cy_TCPWM_Counter_GetCounter(USER_COUNTER_HW, USER_COUNTER_NUM);
    /* Clear SAR result interrupt */
    Cy_HPPASS_SAR_Result_ClearInterrupt(CY_HPPASS_INTR_SAR_RESULT_GROUP_0);
    /* Set the ADC group 0 conversion done flag */
    adc_group0_done_int_flag = true;
}

/*******************************************************************************
* Function Name: user_button_is_pressed
****************************************************************************//**
* Summary:
*  Check if the user button is pressed.
*
* Return:
*  Returns the status of user button.
*
*******************************************************************************/
bool user_button_is_pressed(void)
{
    uint32_t pressCount = 0;

    if(Cy_GPIO_Read(CYBSP_USER_BTN2_PORT, CYBSP_USER_BTN2_NUM) != CYBSP_BTN_PRESSED)
    {
        return false;
    }
    /* Check if User button is pressed */
    while (Cy_GPIO_Read(CYBSP_USER_BTN2_PORT, CYBSP_USER_BTN2_NUM) == CYBSP_BTN_PRESSED)
    {
        /* Wait for 10 ms */
        Cy_SysLib_Delay(10);
        pressCount++;
    }
    /* Add a delay to avoid glitches */
    Cy_SysLib_Delay(10);

    if(10 < pressCount)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/* [] END OF FILE */
