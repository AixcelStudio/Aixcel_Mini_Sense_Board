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


#ifndef SIMULATE_I2C_H
#define SIMULATE_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "at32f4xx.h"

typedef struct __Simulate_I2C_Bus_Typedef
{
    GPIO_Type*       SCL_Port;     /* I2C scl port */
    uint16_t         SCL_Pin;      /* I2C scl pin */
    uint16_t         SCL_Pin_No;   /* I2C scl pin number */
    GPIO_Type*       SDA_Port;     /* I2C sda port */
    uint16_t         SDA_Pin;      /* I2C sda pin */
    uint16_t         SDA_Pin_No;   /* I2C sda pin number */

    uint32_t         Freq;         /* I2C frequency */
    uint32_t         Delay;        /* I2C transmit delay */
	
} Simulate_I2C_Bus_Typedef;

__weak void Delay_us(uint32_t nus);

void Simulate_I2C_Init(Simulate_I2C_Bus_Typedef *I2C); 			           /* Simulate I2C Init */

void Simulate_I2C_Start(Simulate_I2C_Bus_Typedef *I2C);			           /* Simulate I2C Send Start Condition */

void Simulate_I2C_Stop(Simulate_I2C_Bus_Typedef *I2C);			           /* Simulate I2C Send End Condition */

void Simulate_I2C_ACK(Simulate_I2C_Bus_Typedef *I2C);				       /* Simulate I2C Send ACK */

void Simulate_I2C_NACK(Simulate_I2C_Bus_Typedef *I2C);			           /* Simulate I2C Send NACK */

uint8_t Simulate_I2C_Wait_ACK(Simulate_I2C_Bus_Typedef *I2C);		       /* Simulate I2C Wait ACK */

void Simulate_I2C_Send_Byte(Simulate_I2C_Bus_Typedef *I2C, uint8_t byte);  /* Simulate I2C Send One Byte */ 

uint8_t Simulate_I2C_Recv_Byte(Simulate_I2C_Bus_Typedef *I2C); 		       /* Simulate I2C Recv One Byte */

void Simulate_I2C_Master_Transmit(Simulate_I2C_Bus_Typedef *I2C, uint8_t addr, uint8_t *data, uint8_t len);  /* Simulate I2C Send Some Bytes */ 

void Simulate_I2C_Master_Receive(Simulate_I2C_Bus_Typedef *I2C, uint8_t addr, uint8_t *data, uint8_t len);   /* Simulate I2C Recv Some Bytes */


#ifdef __cplusplus
}
#endif

#endif
