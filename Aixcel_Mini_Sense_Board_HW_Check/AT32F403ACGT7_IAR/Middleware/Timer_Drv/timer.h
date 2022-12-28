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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef TIMER_H
#define TIMER_H

#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "common.h"

typedef void (*TIMEPROC)(void);
typedef void (*TIMEPROC_PARAM)(uint8_t param);

#define  FALSE          0
#define  TRUE           1

#define TIMER_POOL_SIZE 45

#define KILLTIMER(T) {KillTimer(T); T=0;}


typedef struct _TIMER_NODE
{
    uint8_t         id;
    uint8_t         busy                   :1;
    uint8_t         type                   :7;
    uint8_t         param;
    uint32_t        need_ticks;
    uint32_t        current_ticks;
    TIMEPROC        func;
    TIMEPROC_PARAM  func_param;
} TIMER_NODE;

typedef struct
{
    uint32_t        miseconds;
    uint8_t         seconds;
    uint8_t         minutes;
    uint32_t        hours;
} TIM_CLOCK;

extern volatile TIM_CLOCK TIM_Clock;


/**************************************************timer��غ�������**************************************************/

/*  
* @fuction:      ISR_PIT0_1ms
* @brief:        PIT0��ʱ���жϴ��� 
* @param[in]:    None
* @param[out]:   None
* @return:       None
* @others:       PIT0��ʱ���ж���Ӧ���� 
*/

void ISR_PIT0_1ms(void);

/*  
* @fuction:      Timer_HAL_Init
* @brief:        ��ʱ���س�ʼ�� 
* @param[in]:    None
* @param[out]:   None
* @return:       None
* @others:       ��ʱ���س�ʼ��ʱʹ��
*/

void TimerPoolInit(void);



/*  
* @fuction:      SetTimer
* @brief:        ��ʱ���趨������һ���¶�ʱ�������趨��ʱʱ���Լ���Ӧ���û����� 
* @param[in]:    a_elapse: ��ʱʱ��(��λ������), a_event_func: ���ﶨʱʱ���ϣ��ִ�еĺ���
* @param[out]:   None
* @return:       ��ʱ�����
* @others:       ��ʱ���趨ʱʹ��
*/

uint8_t SetTimer(uint32_t a_elapse, TIMEPROC a_event_func);



/*  
* @fuction:      KillTimer
* @brief:        ��ʱ��ɾ����ɾ��ָ����ŵ���ʱ�� 
* @param[in]:    a_timer_handle: ��ʱ�����
* @param[out]:   None
* @return:       None
* @others:       ��ʱ��ɾ��ʱʹ��
*/

void KillTimer(uint8_t a_timer_handle);



/*  
* @fuction:      ResetTimer
* @brief:        ��ʱ�����ã���ָ����ŵ���ʱ������һ���µĶ�ʱʱ��
* @param[in]:    a_timer_handle: ��ʱ�����,  a_new_elapse: �µĶ�ʱʱ��
* @param[out]:   None
* @return:       None
* @others:       ��ʱ������ʱʹ��
*/

void ResetTimer(uint8_t a_timer_handle, uint32_t a_new_elapse);



/*  
* @fuction:      TimerEventProcess
* @brief:        ��ʱ����ѯִ�У���������ʱ���Ƿ�ﵽ��ʱʱ�䣬����ﵽ��ִ�иö�ʱ����Ӧ���û�����
* @param[in]:    None
* @param[out]:   None
* @return:       None
* @others:       ��ʱ����ѯִ��ʱʹ��
*/

void TimerEventProcess(void);




#ifdef __cplusplus
}
#endif

#endif /* TIMER_H */
