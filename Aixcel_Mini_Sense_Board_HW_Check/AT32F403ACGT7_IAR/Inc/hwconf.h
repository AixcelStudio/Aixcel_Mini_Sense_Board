/*
******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2022 Aixcel Co.,Ltd</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of Aixcel nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

#ifndef HWCONF_H
#define HWCONF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "at32f4xx.h"
#include <stdio.h>
#include "common.h"

//#define  WWDG_ENABLE
#define  TIM2_ENABLE
#define  TIM3_ENABLE
#define  TIM4_ENABLE

#define  USART1_ENABLE
#define  USART2_ENABLE
#define  ADC1_ENABLE
#define  CAN1_ENABLE

#ifdef USART1_ENABLE
#define  USART1_RX_BUF_CELL_CNT           32U
#define  USART1_RX_BUF_CELL_SIZE          16U
 
#define  USART1_TX_BUF_CELL_CNT           32U
#define  USART1_TX_BUF_CELL_SIZE          16U
#endif

#ifdef USART2_ENABLE
#define  USART2_RX_BUF_CELL_CNT           32U
#define  USART2_RX_BUF_CELL_SIZE          16U

#define  USART2_TX_BUF_CELL_CNT           32U
#define  USART2_TX_BUF_CELL_SIZE          16U
#endif

#ifdef USART3_ENABLE
#define  USART3_RX_BUF_CELL_CNT           32U
#define  USART3_RX_BUF_CELL_SIZE          16U

#define  USART3_TX_BUF_CELL_CNT           32U
#define  USART3_TX_BUF_CELL_SIZE          16U
#endif

#ifdef UART4_ENABLE
#define  UART4_RX_BUF_CELL_CNT            32U
#define  UART4_RX_BUF_CELL_SIZE           16U

#define  UART4_TX_BUF_CELL_CNT            64U
#define  UART4_TX_BUF_CELL_SIZE           16U
#endif

#ifdef UART5_ENABLE
#define  UART5_RX_BUF_CELL_CNT            32U
#define  UART5_RX_BUF_CELL_SIZE           16U

#define  UART5_TX_BUF_CELL_CNT            32U
#define  UART5_TX_BUF_CELL_SIZE           16U
#endif
    
#define  NVIC_PRIORITYGROUP_0     0x00000007U /*!< 0 bits for pre-emption priority
                                                   4 bits for subpriority */
#define  NVIC_PRIORITYGROUP_1     0x00000006U /*!< 1 bits for pre-emption priority
                                                   3 bits for subpriority */
#define  NVIC_PRIORITYGROUP_2     0x00000005U /*!< 2 bits for pre-emption priority
                                                   2 bits for subpriority */
#define  NVIC_PRIORITYGROUP_3     0x00000004U /*!< 3 bits for pre-emption priority
                                                   1 bits for subpriority */
#define  NVIC_PRIORITYGROUP_4     0x00000003U /*!< 4 bits for pre-emption priority
                                                   0 bits for subpriority */

#define  FREQ_1MHz                   1000000U

#define  FREQ_8MHz                   8000000U

#define  FREQ_12MHz                 12000000U

#define  FREQ_20MHz                 20000000U

#define  FREQ_100KHz                  100000U

#define  FREQ_400KHz                  400000U


//#define  USE_DMA_TX_USART1_DATA

/* USE_DMA_RX_USART1_DATA 和 USE_PWM_CTRL_RGB_STRIP_1ST 只能二选一*/
//#define  USE_DMA_RX_USART1_DATA

/* USE_DMA_TX_USART2_DATA 和 USE_PWM_CTRL_RGB_STRIP_2ND 只能二选一*/
//#define  USE_DMA_TX_USART2_DATA
                                                       
/* USE_DMA_TX_USART3_DATA 和 USE_PWM_CTRL_RGB_STRIP_3RD 只能二选一*/
//#define  USE_DMA_TX_USART3_DATA
                                                       
/* USE_DMA_RX_USART3_DATA 和 USE_PWM_CTRL_RGB_STRIP_4TH 只能二选一*/
//#define  USE_DMA_RX_USART3_DATA

#define  USE_PWM_CTRL_RGB_STRIP_1ST

#define  USE_PWM_CTRL_RGB_STRIP_2ND

#define  USE_PWM_CTRL_RGB_STRIP_3RD

#define  USE_PWM_CTRL_RGB_STRIP_4TH
  
/* USART Pin defines ---------------------------------------------------------*/
#define  USART1_Tx_Pin            GPIO_Pins_9
#define  USART1_Rx_Pin            GPIO_Pins_10
#define  USART1_GPIO_Port         GPIOA

#define  USART2_Tx_Pin            GPIO_Pins_2
#define  USART2_Rx_Pin            GPIO_Pins_3
#define  USART2_GPIO_Port         GPIOA

/* CAN Pin defines -----------------------------------------------------------*/
#define  CAN1_RX_Pin              GPIO_Pins_11
#define  CAN1_TX_Pin              GPIO_Pins_12
#define  CAN1_GPIO_Port           GPIOA

/* Pin defines ---------------------------------------------------------------*/
#define  M485_DE_Pin              GPIO_Pins_13
#define  M485_DE_GPIO_Port        GPIOC

#define  SYS_LED_Pin              GPIO_Pins_15
#define  SYS_LED_GPIO_Port        GPIOA

/* TIO Pin defines ---------------------------------------------------------------*/
#define  MCU_TIO_1_Pin            GPIO_Pins_4
#define  MCU_TIO_1_GPIO_Port      GPIOB
#define  MCU_TIO_1_Pin_Source     GPIO_PinsSource4
#define  MCU_TIO_1_Port_Source    GPIO_PortSourceGPIOB

#define  MCU_TIO_2_Pin            GPIO_Pins_5
#define  MCU_TIO_2_GPIO_Port      GPIOB
#define  MCU_TIO_2_Pin_Source     GPIO_PinsSource5
#define  MCU_TIO_2_Port_Source    GPIO_PortSourceGPIOB

#define  MCU_TIO_3_Pin            GPIO_Pins_0
#define  MCU_TIO_3_GPIO_Port      GPIOB
#define  MCU_TIO_3_Pin_Source     GPIO_PinsSource0
#define  MCU_TIO_3_Port_Source    GPIO_PortSourceGPIOB

#define  MCU_TIO_4_Pin            GPIO_Pins_1
#define  MCU_TIO_4_GPIO_Port      GPIOB
#define  MCU_TIO_4_Pin_Source     GPIO_PinsSource1
#define  MCU_TIO_4_Port_Source    GPIO_PortSourceGPIOB

/* AIO Pin defines ---------------------------------------------------------------*/
#define  MCU_AIO_1_Pin            GPIO_Pins_4
#define  MCU_AIO_1_GPIO_Port      GPIOA
#define  MCU_AIO_1_Pin_Source     GPIO_PinsSource4
#define  MCU_AIO_1_Port_Source    GPIO_PortSourceGPIOA

#define  MCU_AIO_2_Pin            GPIO_Pins_5
#define  MCU_AIO_2_GPIO_Port      GPIOA
#define  MCU_AIO_2_Pin_Source     GPIO_PinsSource5
#define  MCU_AIO_2_Port_Source    GPIO_PortSourceGPIOA

#define  MCU_AIO_3_Pin            GPIO_Pins_6
#define  MCU_AIO_3_GPIO_Port      GPIOA
#define  MCU_AIO_3_Pin_Source     GPIO_PinsSource6
#define  MCU_AIO_3_Port_Source    GPIO_PortSourceGPIOA

#define  MCU_AIO_4_Pin            GPIO_Pins_7
#define  MCU_AIO_4_GPIO_Port      GPIOA
#define  MCU_AIO_4_Pin_Source     GPIO_PinsSource7
#define  MCU_AIO_4_Port_Source    GPIO_PortSourceGPIOA

/* SCL Pin defines ---------------------------------------------------------------*/
#define  MCU_SCL_1_Pin            GPIO_Pins_12
#define  MCU_SCL_1_GPIO_Port      GPIOB

#define  MCU_SCL_2_Pin            GPIO_Pins_13
#define  MCU_SCL_2_GPIO_Port      GPIOB

#define  MCU_SCL_3_Pin            GPIO_Pins_14
#define  MCU_SCL_3_GPIO_Port      GPIOB

#define  MCU_SCL_4_Pin            GPIO_Pins_15
#define  MCU_SCL_4_GPIO_Port      GPIOB

#define  MCU_SCL_5_Pin            GPIO_Pins_15
#define  MCU_SCL_5_GPIO_Port      GPIOC

#define  MCU_SCL_6_Pin            GPIO_Pins_14
#define  MCU_SCL_6_GPIO_Port      GPIOC

#define  MCU_SCL_7_Pin            GPIO_Pins_8
#define  MCU_SCL_7_GPIO_Port      GPIOA

#define  MCU_SCL_8_Pin            GPIO_Pins_3
#define  MCU_SCL_8_GPIO_Port      GPIOB

/* SDA Pin defines ---------------------------------------------------------------*/
#define  MCU_SDA_1_Pin            GPIO_Pins_6
#define  MCU_SDA_1_GPIO_Port      GPIOB

#define  MCU_SDA_2_Pin            GPIO_Pins_7
#define  MCU_SDA_2_GPIO_Port      GPIOB

#define  MCU_SDA_3_Pin            GPIO_Pins_8
#define  MCU_SDA_3_GPIO_Port      GPIOB

#define  MCU_SDA_4_Pin            GPIO_Pins_9
#define  MCU_SDA_4_GPIO_Port      GPIOB

#define  MCU_SDA_5_Pin            GPIO_Pins_0
#define  MCU_SDA_5_GPIO_Port      GPIOA

#define  MCU_SDA_6_Pin            GPIO_Pins_1
#define  MCU_SDA_6_GPIO_Port      GPIOA

#define  MCU_SDA_7_Pin            GPIO_Pins_10
#define  MCU_SDA_7_GPIO_Port      GPIOB

#define  MCU_SDA_8_Pin            GPIO_Pins_11
#define  MCU_SDA_8_GPIO_Port      GPIOB

/* GPIO Pin function defines -------------------------------------------------*/
#define  SYS_LED_ON()             (SYS_LED_GPIO_Port->BRE = SYS_LED_Pin)
#define  SYS_LED_OFF()            (SYS_LED_GPIO_Port->BSRE = SYS_LED_Pin)
#define  SYS_LED_TOGGLE()         (SYS_LED_GPIO_Port->OPTDT ^= SYS_LED_Pin)

#define  M485_DE_Pin_PULL_DOWN()  (M485_DE_GPIO_Port->BRE = M485_DE_Pin)
#define  M485_DE_Pin_PULL_UP()    (M485_DE_GPIO_Port->BSRE = M485_DE_Pin)
#define  M485_DE_Pin_TOGGLE()     (M485_DE_GPIO_Port->OPTDT ^= M485_DE_Pin)

#define  MCU_TIO_1_PULL_DOWN()    (MCU_TIO_1_GPIO_Port->BRE  = MCU_TIO_1_Pin)
#define  MCU_TIO_1_PULL_UP()      (MCU_TIO_1_GPIO_Port->BSRE = MCU_TIO_1_Pin)
#define  MCU_TIO_1_TOGGLE()       (MCU_TIO_1_GPIO_Port->OPTDT ^= MCU_TIO_1_Pin)

#define  MCU_TIO_2_PULL_DOWN()    (MCU_TIO_2_GPIO_Port->BRE  = MCU_TIO_2_Pin)
#define  MCU_TIO_2_PULL_UP()      (MCU_TIO_2_GPIO_Port->BSRE = MCU_TIO_2_Pin)
#define  MCU_TIO_2_TOGGLE()       (MCU_TIO_2_GPIO_Port->OPTDT ^= MCU_TIO_2_Pin)

#define  MCU_TIO_3_PULL_DOWN()    (MCU_TIO_3_GPIO_Port->BRE  = MCU_TIO_3_Pin)
#define  MCU_TIO_3_PULL_UP()      (MCU_TIO_3_GPIO_Port->BSRE = MCU_TIO_3_Pin)
#define  MCU_TIO_3_TOGGLE()       (MCU_TIO_3_GPIO_Port->OPTDT ^= MCU_TIO_3_Pin)

#define  MCU_TIO_4_PULL_DOWN()    (MCU_TIO_4_GPIO_Port->BRE  = MCU_TIO_4_Pin)
#define  MCU_TIO_4_PULL_UP()      (MCU_TIO_4_GPIO_Port->BSRE = MCU_TIO_4_Pin)
#define  MCU_TIO_4_TOGGLE()       (MCU_TIO_4_GPIO_Port->OPTDT ^= MCU_TIO_4_Pin)

#define  MCU_AIO_1_PULL_DOWN()    (MCU_AIO_1_GPIO_Port->BRE  = MCU_AIO_1_Pin)
#define  MCU_AIO_1_PULL_UP()      (MCU_AIO_1_GPIO_Port->BSRE = MCU_AIO_1_Pin)
#define  MCU_AIO_1_TOGGLE()       (MCU_AIO_1_GPIO_Port->OPTDT ^= MCU_AIO_1_Pin)

#define  MCU_AIO_2_PULL_DOWN()    (MCU_AIO_2_GPIO_Port->BRE  = MCU_AIO_2_Pin)
#define  MCU_AIO_2_PULL_UP()      (MCU_AIO_2_GPIO_Port->BSRE = MCU_AIO_2_Pin)
#define  MCU_AIO_2_TOGGLE()       (MCU_AIO_2_GPIO_Port->OPTDT ^= MCU_AIO_2_Pin)

#define  MCU_AIO_3_PULL_DOWN()    (MCU_AIO_3_GPIO_Port->BRE  = MCU_AIO_3_Pin)
#define  MCU_AIO_3_PULL_UP()      (MCU_AIO_3_GPIO_Port->BSRE = MCU_AIO_3_Pin)
#define  MCU_AIO_3_TOGGLE()       (MCU_AIO_3_GPIO_Port->OPTDT ^= MCU_AIO_3_Pin)

#define  MCU_AIO_4_PULL_DOWN()    (MCU_AIO_4_GPIO_Port->BRE  = MCU_AIO_4_Pin)
#define  MCU_AIO_4_PULL_UP()      (MCU_AIO_4_GPIO_Port->BSRE = MCU_AIO_4_Pin)
#define  MCU_AIO_4_TOGGLE()       (MCU_AIO_4_GPIO_Port->OPTDT ^= MCU_AIO_4_Pin)

#define  MCU_SCL_1_PULL_DOWN()    (MCU_SCL_1_GPIO_Port->BRE  = MCU_SCL_1_Pin)
#define  MCU_SCL_1_PULL_UP()      (MCU_SCL_1_GPIO_Port->BSRE = MCU_SCL_1_Pin)
#define  MCU_SCL_1_TOGGLE()       (MCU_SCL_1_GPIO_Port->OPTDT ^= MCU_SCL_1_Pin)

#define  MCU_SCL_2_PULL_DOWN()    (MCU_SCL_2_GPIO_Port->BRE  = MCU_SCL_2_Pin)
#define  MCU_SCL_2_PULL_UP()      (MCU_SCL_2_GPIO_Port->BSRE = MCU_SCL_2_Pin)
#define  MCU_SCL_2_TOGGLE()       (MCU_SCL_2_GPIO_Port->OPTDT ^= MCU_SCL_2_Pin)

#define  MCU_SCL_3_PULL_DOWN()    (MCU_SCL_3_GPIO_Port->BRE  = MCU_SCL_3_Pin)
#define  MCU_SCL_3_PULL_UP()      (MCU_SCL_3_GPIO_Port) = MCU_SCL_3_Pin)
#define  MCU_SCL_3_TOGGLE()       (MCU_SCL_3_GPIO_Port->OPTDT ^= MCU_SCL_3_Pin)

#define  MCU_SCL_4_PULL_DOWN()    (MCU_SCL_4_GPIO_Port->BRE  = MCU_SCL_4_Pin)
#define  MCU_SCL_4_PULL_UP()      (MCU_SCL_4_GPIO_Port->BSRE = MCU_SCL_4_Pin)
#define  MCU_SCL_4_TOGGLE()       (MCU_SCL_4_GPIO_Port->OPTDT ^= MCU_SCL_4_Pin)

#define  MCU_SCL_5_PULL_DOWN()    (MCU_SCL_5_GPIO_Port->BRE  = MCU_SCL_5_Pin)
#define  MCU_SCL_5_PULL_UP()      (MCU_SCL_5_GPIO_Port->BSRE = MCU_SCL_5_Pin)
#define  MCU_SCL_5_TOGGLE()       (MCU_SCL_5_GPIO_Port->OPTDT ^= MCU_SCL_5_Pin)

#define  MCU_SCL_6_PULL_DOWN()    (MCU_SCL_6_GPIO_Port->BRE  = MCU_SCL_6_Pin)
#define  MCU_SCL_6_PULL_UP()      (MCU_SCL_6_GPIO_Port->BSRE = MCU_SCL_6_Pin)
#define  MCU_SCL_6_TOGGLE()       (MCU_SCL_6_GPIO_Port->OPTDT ^= MCU_SCL_6_Pin)

#define  MCU_SCL_7_PULL_DOWN()    (MCU_SCL_7_GPIO_Port->BRE  = MCU_SCL_7_Pin)
#define  MCU_SCL_7_PULL_UP()      (MCU_SCL_7_GPIO_Port->BSRE = MCU_SCL_7_Pin)
#define  MCU_SCL_7_TOGGLE()       (MCU_SCL_7_GPIO_Port->OPTDT ^= MCU_SCL_7_Pin)

#define  MCU_SCL_8_PULL_DOWN()    (MCU_SCL_8_GPIO_Port->BRE  = MCU_SCL_8_Pin)
#define  MCU_SCL_8_PULL_UP()      (MCU_SCL_8_GPIO_Port->BSRE = MCU_SCL_8_Pin)
#define  MCU_SCL_8_TOGGLE()       (MCU_SCL_8_GPIO_Port->OPTDT ^= MCU_SCL_8_Pin)

#define  MCU_SDA_1_PULL_DOWN()    (MCU_SDA_1_GPIO_Port->BRE  = MCU_SDA_1_Pin)
#define  MCU_SDA_1_PULL_UP()      (MCU_SDA_1_GPIO_Port->BSRE = MCU_SDA_1_Pin)
#define  MCU_SDA_1_TOGGLE()       (MCU_SDA_1_GPIO_Port->OPTDT ^= MCU_SDA_1_Pin)

#define  MCU_SDA_2_PULL_DOWN()    (MCU_SDA_2_GPIO_Port->BRE  = MCU_SDA_2_Pin)
#define  MCU_SDA_2_PULL_UP()      (MCU_SDA_2_GPIO_Port->BSRE = MCU_SDA_2_Pin)
#define  MCU_SDA_2_TOGGLE()       (MCU_SDA_2_GPIO_Port->OPTDT ^= MCU_SDA_2_Pin)

#define  MCU_SDA_3_PULL_DOWN()    (MCU_SDA_3_GPIO_Port->BRE  = MCU_SDA_3_Pin)
#define  MCU_SDA_3_PULL_UP()      (MCU_SDA_3_GPIO_Port->BSRE = MCU_SDA_3_Pin)
#define  MCU_SDA_3_TOGGLE()       (MCU_SDA_3_GPIO_Port->OPTDT ^= MCU_SDA_3_Pin)

#define  MCU_SDA_4_PULL_DOWN()    (MCU_SDA_4_GPIO_Port->BRE  = MCU_SDA_4_Pin)
#define  MCU_SDA_4_PULL_UP()      (MCU_SDA_4_GPIO_Port->BSRE = MCU_SDA_4_Pin)
#define  MCU_SDA_4_TOGGLE()       (MCU_SDA_4_GPIO_Port->OPTDT ^= MCU_SDA_4_Pin)

#define  MCU_SDA_5_PULL_DOWN()    (MCU_SDA_5_GPIO_Port->BRE  = MCU_SDA_5_Pin)
#define  MCU_SDA_5_PULL_UP()      (MCU_SDA_5_GPIO_Port->BSRE = MCU_SDA_5_Pin)
#define  MCU_SDA_5_TOGGLE()       (MCU_SDA_5_GPIO_Port->OPTDT ^= MCU_SDA_5_Pin)

#define  MCU_SDA_6_PULL_DOWN()    (MCU_SDA_6_GPIO_Port->BRE  = MCU_SDA_6_Pin)
#define  MCU_SDA_6_PULL_UP()      (MCU_SDA_6_GPIO_Port->BSRE = MCU_SDA_6_Pin)
#define  MCU_SDA_6_TOGGLE()       (MCU_SDA_6_GPIO_Port->OPTDT ^= MCU_SDA_6_Pin)

#define  MCU_SDA_7_PULL_DOWN()    (MCU_SDA_7_GPIO_Port->BRE  = MCU_SDA_7_Pin)
#define  MCU_SDA_7_PULL_UP()      (MCU_SDA_7_GPIO_Port->BSRE = MCU_SDA_7_Pin)
#define  MCU_SDA_7_TOGGLE()       (MCU_SDA_7_GPIO_Port->OPTDT ^= MCU_SDA_7_Pin)

#define  MCU_SDA_8_PULL_DOWN()    (MCU_SDA_8_GPIO_Port->BRE  = MCU_SDA_8_Pin)
#define  MCU_SDA_8_PULL_UP()      (MCU_SDA_8_GPIO_Port->BSRE = MCU_SDA_8_Pin)
#define  MCU_SDA_8_TOGGLE()       (MCU_SDA_8_GPIO_Port->OPTDT ^= MCU_SDA_8_Pin)


void gpio_init(GPIO_Type* GPIOx, GPIOMode_Type GPIO_Mode, GPIOMaxSpeed_Type GPIO_MaxSpeed, uint16_t GPIO_Pins);

void System_GPIO_Init(void);

void System_NVIC_Init(void);

void System_Misc_Init(void);

void System_EXTI_Init(void);

void System_DMA_Init(void);

void System_RTC_Init(void);

void System_USART1_Init(uint32_t baud_rate);

void System_USART2_Init(uint32_t baud_rate);

void System_USART3_Init(uint32_t baud_rate);

void USART2_EN_M485_Tx(void);

void USART2_EN_M485_Rx(void);

void USB_CDC_USART_Init(uint32_t baud_rate);

void System_TMR1_Init(void);

void System_TMR2_Init(void);

void System_TMR3_Init(void);

void System_TMR4_Init(void);

void System_TMR6_Init(void);

void System_DAC_OUT1_Init(void);

void System_ADC1_Init(void);

void System_SPI1_Init(uint32_t baud_rate);

void System_SPI2_Init(uint32_t baud_rate);

void System_I2C1_Init(uint32_t baud_rate);

void System_CAN1_Init(CAN_Baudrate_Typedef baudrate);

void System_CAN2_Init(CAN_Baudrate_Typedef baudrate);

uint8_t System_CAN_Start(CAN_Type* can_periph);

uint8_t System_CAN_Stop(CAN_Type* can_periph);

void System_WWDG_Init(void);

void System_WWDG_Disable(void);

void EXTI0_IRQ_Callback(void);

void EXTI1_IRQ_Callback(void);

void EXTI2_IRQ_Callback(void);

void EXTI3_IRQ_Callback(void);

void EXTI4_IRQ_Callback(void);

void EXTI5_IRQ_Callback(void);

void EXTI6_IRQ_Callback(void);

void EXTI7_IRQ_Callback(void);

void EXTI15_10_IRQ_Callback(void);


#ifdef __cplusplus
}
#endif

#endif /* HWCONF_H */
