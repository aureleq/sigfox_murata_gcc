/******************************************************************************
 * @file    mlm32l0xx_hw_conf.h
 * @author  MCD Application Team
 * @version V1.1.0
 * @date    30-April-2018
 * @brief   contains hardware configuration Macros and Constants
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
#ifndef __HW_CONF_L0_H__
#define __HW_CONF_L0_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

#define RADIO_DIO_4
/* #define RADIO_DIO_5 */

/*  I/O definition */
   
#define RADIO_RESET_PORT                          GPIOC
#define RADIO_RESET_PIN                           GPIO_PIN_0

#define RADIO_MOSI_PORT                           GPIOA
#define RADIO_MOSI_PIN                            GPIO_PIN_7

#define RADIO_MISO_PORT                           GPIOA
#define RADIO_MISO_PIN                            GPIO_PIN_6

#define RADIO_SCLK_PORT                           GPIOB
#define RADIO_SCLK_PIN                            GPIO_PIN_3

#define RADIO_NSS_PORT                            GPIOA
#define RADIO_NSS_PIN                             GPIO_PIN_15
#define RADIO_NSS_GPIO_CLK_ENABLE()               __GPIOA_CLK_ENABLE()

#define RADIO_DIO_0_PORT                          GPIOB
#define RADIO_DIO_0_PIN                           GPIO_PIN_4

#define RADIO_DIO_1_PORT                          GPIOB
#define RADIO_DIO_1_PIN                           GPIO_PIN_1

#define RADIO_DIO_2_PORT                          GPIOB
#define RADIO_DIO_2_PIN                           GPIO_PIN_0

#define RADIO_DIO_3_PORT                          GPIOC
#define RADIO_DIO_3_PIN                           GPIO_PIN_13

#define RADIO_DIO_4_PORT                          GPIOA
#define RADIO_DIO_4_PIN                           GPIO_PIN_5

#define RADIO_DIO_5_PORT                          GPIOA
#define RADIO_DIO_5_PIN                           GPIO_PIN_4

#define RADIO_TCXO_VCC_PORT                       GPIOA
#define RADIO_TCXO_VCC_PIN                        GPIO_PIN_12

#define RADIO_ANT_SWITCH_PORT_RX                  GPIOA /* CRF1 */
#define RADIO_ANT_SWITCH_PIN_RX                   GPIO_PIN_1

#define RADIO_ANT_SWITCH_PORT_TX_BOOST            GPIOC /* CRF3 */
#define RADIO_ANT_SWITCH_PIN_TX_BOOST             GPIO_PIN_1

#define RADIO_ANT_SWITCH_PORT_TX_RFO              GPIOC /* CRF2 */
#define RADIO_ANT_SWITCH_PIN_TX_RFO               GPIO_PIN_2

/*  SPI MACRO redefinition */

#define SPI1_AF                          GPIO_AF0_SPI1  

/* #define SPIx                             SPI1 */
#define SPIx_CLK_ENABLE()                __HAL_RCC_SPI1_CLK_ENABLE()
#define DMAx_CLK_ENABLE()                __HAL_RCC_DMA1_CLK_ENABLE()

/* Definition for SPIx's DMA */
/* to set on TIM2_CH2*/
#define SPIx_TX_DMA_CHANNEL              DMA1_Channel3
/* to set on timer 2*/
#define SPIx_TX_DMA_REQUEST              DMA_REQUEST_8 

/* Definition for SPIx's NVIC */
#define SPIx_DMA_TX_IRQn                 DMA1_Channel2_3_IRQn
#define SPIx_DMA_TX_IRQHandler           DMA1_Channel2_3_IRQHandler

/* Definition for SPIx's FREQUENCY */
#define SPIx_BAUDRATE                   16000000 /* 16M Sigfox, 10M Lora */

/* Definition for TIMx Channel Pins */
#define TIMx                           TIM2
#define TIMx_CLK_ENABLE()              __HAL_RCC_TIM2_CLK_ENABLE()
#define TIMx_CHANNEL_GPIO_PORT()       __HAL_RCC_GPIOA_CLK_ENABLE();
#define TIMx_GPIO_PORT_CHANNEL1        RADIO_NSS_PORT
#define TIMx_GPIO_PIN_CHANNEL1         RADIO_NSS_PIN
#define TIMx_GPIO_AF_CHANNEL1          GPIO_AF5_TIM2


/* ADC MACRO redefinition */

#define ADCCLK_ENABLE()                 __HAL_RCC_ADC1_CLK_ENABLE() ;
#define ADCCLK_DISABLE()                __HAL_RCC_ADC1_CLK_DISABLE() ;



/* --------------------------- RTC HW definition -------------------------------- */

#define RTC_OUTPUT       DBG_RTC_OUTPUT

#define RTC_Alarm_IRQn              RTC_IRQn
/* --------------------------- USART HW definition -------------------------------*/


#define UARTX                           LPUART1
#define UARTX_CLK_ENABLE()               __LPUART1_CLK_ENABLE()
#define UARTX_RX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()
#define UARTX_TX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE() 

#define UARTX_FORCE_RESET()             __LPUART1_FORCE_RESET()
#define UARTX_RELEASE_RESET()           __LPUART1_RELEASE_RESET()

#define UARTX_TX_PIN                    GPIO_PIN_2
#define UARTX_TX_GPIO_PORT              GPIOA
#define UARTX_TX_AF                     GPIO_AF6_LPUART1
#define UARTX_RX_PIN                    GPIO_PIN_3
#define UARTX_RX_GPIO_PORT              GPIOA
#define UARTX_RX_AF                     GPIO_AF6_LPUART1

/* Definition for USARTx's NVIC */
#define UARTX_IRQn                      LPUART1_IRQn
#define UARTX_IRQHandler                LPUART1_IRQHandler


#ifdef __cplusplus
}
#endif

#endif /* __HW_CONF_L0_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
