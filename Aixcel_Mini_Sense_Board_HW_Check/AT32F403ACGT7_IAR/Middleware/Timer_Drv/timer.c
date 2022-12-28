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

#include "timer.h"

volatile TIM_CLOCK TIM_Clock = {0};

TIMER_NODE General_Timers[TIMER_POOL_SIZE];


/*  
* @fuction:      ISR_PIT0_1ms
* @brief:        PIT0��ʱ���жϴ��� 
* @param[in]:    None
* @param[out]:   None
* @return:       None
* @others:       PIT0��ʱ���ж���Ӧ���� 
*/

void ISR_PIT0_1ms(void)
{
    uint32_t i;
    
    for(i=0; i<TIMER_POOL_SIZE; i++)
    {
        if(General_Timers[i].busy == 1)
        {
            General_Timers[i].current_ticks ++;
            if(General_Timers[i].current_ticks > 60000)
            {
                General_Timers[i].current_ticks = 60001;
            }
        }
    }


    TIM_Clock.miseconds ++;
    
    if(TIM_Clock.miseconds >= 1000)
    {
        TIM_Clock.miseconds = 0;
        TIM_Clock.seconds ++;
        
        if(TIM_Clock.seconds >= 60)
        {
            TIM_Clock.seconds = 0;
            TIM_Clock.minutes ++;
            
            if(TIM_Clock.minutes >= 60)
            {
                TIM_Clock.minutes = 0;
                TIM_Clock.hours ++;
            }
        }
    }
}

/*  
* @fuction:      Timer_HAL_Init
* @brief:        ��ʱ��Ӳ��������ʼ�� 
* @param[in]:    None
* @param[out]:   None
* @return:       None
* @others:       ��ʱ��Ӳ��������ʼ��ʱʹ��
*/

static void Timer_HAL_Init(void)
{    
    ;
}


/*  
* @fuction:      TimerPoolInit
* @brief:        ��ʱ���س�ʼ�� 
* @param[in]:    None
* @param[out]:   None
* @return:       None
* @others:       ��ʱ���س�ʼ��ʱʹ��
*/

void TimerPoolInit(void)
{
    uint32_t i;

    Timer_HAL_Init();

    for(i=0;i<TIMER_POOL_SIZE;i++)
    {
        General_Timers[i].busy = 0;
        General_Timers[i].id = i+1;
        General_Timers[i].func = NULL;        
        General_Timers[i].current_ticks = 0;
        General_Timers[i].need_ticks = 0;
        General_Timers[i].type = 0;
    }
}


/*  
* @fuction:      SetTimer
* @brief:        ��ʱ���趨������һ���¶�ʱ�������趨��ʱʱ���Լ���Ӧ���û����� 
* @param[in]:    a_elapse: ��ʱʱ��(��λ������), a_event_func: ���ﶨʱʱ���ϣ��ִ�еĺ���
* @param[out]:   None
* @return:       ��ʱ�����
* @others:       ��ʱ���趨ʱʹ��
*/

uint8_t SetTimer(uint32_t a_elapse, TIMEPROC a_event_func)
{
    uint32_t i;
    uint8_t ret = 0;

    for(i=0; i<TIMER_POOL_SIZE; i++)
    {
        if(General_Timers[i].busy == 0)
        {            
            if(a_elapse <= 60000)
            {                
                General_Timers[i].busy = 1;
                General_Timers[i].type = 0;
                General_Timers[i].need_ticks = a_elapse;
                General_Timers[i].func = a_event_func;
                
                ret = General_Timers[i].id;
            }           
            
            break;
        }         
    }

    return ret;
}


/*  
* @fuction:      KillTimer
* @brief:        ��ʱ��ɾ����ɾ��ָ����ŵ���ʱ�� 
* @param[in]:    a_timer_handle: ��ʱ�����
* @param[out]:   None
* @return:       None
* @others:       ��ʱ��ɾ��ʱʹ��
*/

void KillTimer(uint8_t a_timer_handle)
{
    if(a_timer_handle == 0)
    {
        return;
    }

    if(a_timer_handle <= TIMER_POOL_SIZE)
    {
        General_Timers[a_timer_handle-1].busy = 0;
        General_Timers[a_timer_handle-1].type = 0;
        General_Timers[a_timer_handle-1].param = 0;
        General_Timers[a_timer_handle-1].need_ticks = 0;        
        General_Timers[a_timer_handle-1].current_ticks = 0;
        General_Timers[a_timer_handle-1].func = NULL;        
    }
}


/*  
* @fuction:      ResetTimer
* @brief:        ��ʱ�����ã���ָ����ŵ���ʱ������һ���µĶ�ʱʱ��
* @param[in]:    a_timer_handle: ��ʱ�����,  a_new_elapse: �µĶ�ʱʱ��
* @param[out]:   None
* @return:       None
* @others:       ��ʱ������ʱʹ��
*/

void ResetTimer(uint8_t a_timer_handle, uint32_t a_new_elapse)
{

    if(a_timer_handle == 0)
    {
        return;
    }

    if(a_timer_handle <= TIMER_POOL_SIZE)
    {
        if(a_new_elapse <= 60000)
        {
            General_Timers[a_timer_handle-1].busy = 0;
            General_Timers[a_timer_handle-1].need_ticks = a_new_elapse;
            General_Timers[a_timer_handle-1].current_ticks = 0;
            General_Timers[a_timer_handle-1].busy = 1;
        }
    }
}


/*  
* @fuction:      TimerEventProcess
* @brief:        ��ʱ����ѯִ�У���������ʱ���Ƿ�ﵽ��ʱʱ�䣬����ﵽ��ִ�иö�ʱ����Ӧ���û�����
* @param[in]:    None
* @param[out]:   None
* @return:       None
* @others:       ��ʱ����ѯִ��ʱʹ��
*/

void TimerEventProcess(void)
{
    uint32_t i;
    for(i=0; i<TIMER_POOL_SIZE; i++)
    {
        if((General_Timers[i].busy != 0) && (General_Timers[i].current_ticks >= General_Timers[i].need_ticks))
        {
            if(General_Timers[i].func != NULL)
            {
                if(General_Timers[i].type == 0)
                {
                    General_Timers[i].func();
                }
            }
            else if(General_Timers[i].func_param != NULL)
            {
                if(General_Timers[i].type == 1)
                {
                    General_Timers[i].func_param(General_Timers[i].param);
                }
            }
            
            General_Timers[i].current_ticks = 0;
        }        
    }
}


