/******************************************************************************
 * @file    hw_msp.h
 * @author  MCD Application Team
 * @version V1.1.0
 * @date    30-April-2018
 * @brief   Header for driver hw msp module
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
 * All rights reserved.</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __HW_MSP_H__
#define __HW_MSP_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

#define VDDA_VREFINT_CAL         ((uint32_t) 3000)
#define BAT_CR2032               ((uint32_t) 3000)
#define VDD_BAT                  BAT_CR2032
#define VDD_MIN                  1800

/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/

/*!
 * \brief GPIOs Macro
 */

#define RCC_GPIO_CLK_ENABLE( __GPIO_PORT__ )                   \
do {                                                           \
    switch( __GPIO_PORT__)                                     \
    {                                                          \
      case GPIOA_BASE: __HAL_RCC_GPIOA_CLK_ENABLE(); break;    \
      case GPIOB_BASE: __HAL_RCC_GPIOB_CLK_ENABLE(); break;    \
      case GPIOC_BASE: __HAL_RCC_GPIOC_CLK_ENABLE(); break;    \
      case GPIOD_BASE: __HAL_RCC_GPIOD_CLK_ENABLE(); break;    \
      case GPIOH_BASE: default:  __HAL_RCC_GPIOH_CLK_ENABLE(); \
    }                                                          \
  } while(0)

#define RCC_GPIO_CLK_DISABLE( __GPIO_PORT__ )                   \
do {                                                            \
    switch( __GPIO_PORT__)                                      \
    {                                                           \
      case GPIOA_BASE: __HAL_RCC_GPIOA_CLK_DISABLE(); break;    \
      case GPIOB_BASE: __HAL_RCC_GPIOB_CLK_DISABLE(); break;    \
      case GPIOC_BASE: __HAL_RCC_GPIOC_CLK_DISABLE(); break;    \
      case GPIOD_BASE: __HAL_RCC_GPIOD_CLK_DISABLE(); break;    \
      case GPIOH_BASE: default:  __HAL_RCC_GPIOH_CLK_ENABLE();  \
    }                                                           \
  } while(0)

/* Exported functions ------------------------------------------------------- */ 

/**
 * @brief Get the current temperature
 * @note 
 * @param vtemperature in degreeCelcius( q7.8 )
 * @retval none
 */
uint16_t HW_GetTemperatureLevel( void );

/**
 * @brief value  battery level in mV
 * @note 
 * @param value  battery level in mV
 * @retval none
 */
uint16_t HW_GetBatteryLevel( void );
  
/**
  * @brief This function initializes the hardware
  * @param None
  * @retval None
  */
void HW_Init( void );

/**
 * @brief De-initializes the peripherals
 * @note 
 * @param none
 * @retval none
 */
void HW_DeInit( void );

/**
  * @brief nitializes the HW and enters stop mode
  * @note 
  * @param none
  * @retval none
  */
void HW_EnterStopMode( void);

/**
  * @brief Exits stop mode and Initializes the HW
  * @note 
  * @param none
  * @retval none
  */
void HW_ExitStopMode( void);

/**
  * @brief Swicth HSE as system clock source
  * @note 
  * @param none
  * @retval none
  */
void HW_SetHSEasSysClock( void);

/**
  * @brief Swicth HSI+PLL as system clock source
  * @note 
  * @param none
  * @retval none
  */
void HW_SetHSIasSysClock( void);
  
/**
  * @brief Enters Low Power Sleep Mode
  * @note ARM exists the function when waking up
  * @param none
  * @retval none
  */
void HW_EnterSleepMode( void);

/* ADC */

/**
  * @brief This function De-initializes the ADC
  * @param none
  * @retval none
  */
void HW_AdcInit(  void );

/*!
 * \brief DeInitializes the ADC 
 *
 * \param [IN] none
 */
void HW_AdcDeInit( void );

/*!
 * \brief Read the analogue voltage value
 *
 * \param [IN] Channel to read
 * \retval value    Analogue pin value
 */
uint16_t HW_AdcReadChannel( uint32_t Channel);

/*!
 * \brief Configures the sytem Clock at start-up
 *
 * \param none
 * \retval none
 */
void SystemClock_Config( void );

/**
  * @brief  Configure all GPIO's to Analog input to reduce the power consumption
  * @param  None
  * @retval None
  */
void HW_GpioInit(void);


#ifdef __cplusplus
}
#endif

#endif /* __HW_MSP_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
