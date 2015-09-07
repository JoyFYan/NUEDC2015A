/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
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
/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "stm32f3xx.h"
#include "stm32f3xx_it.h"

/* USER CODE BEGIN 0 */
//#include "oled.h"
#define DT_RISING       ((uint16_t)25) // Dead time rising edge
#define DT_FALLING      ((uint16_t)35) // Dead time falling edge
#define BUCK_PWM_PERIOD ((uint16_t) 45600) /* 120kHz */
#define MAX_ERROR     ((uint16_t)500)
#define MI 0
#define MV 1
#define Stop 2
#define MI1 3

//#define Izero 3101
extern int32_t PI_I(void);
extern int32_t PI_V(void);
//extern int32_t PI_IB(void);
//extern int32_t PI_VB(void);
extern int32_t PI_I2(void);
extern HRTIM_HandleTypeDef hhrtim1;
extern unsigned int cont;
extern unsigned int f;
extern void OLED_P8x16Str(unsigned char x,unsigned char y,unsigned char ch[]);
extern void Reset_PI(void);
uint32_t VoutT=1890,V=1,V1=0,I1=0,I=0,Vrms=0,Irms=0,mdelay=0,Itemp=0,Iset=3262,Vtemp=0,V1temp=0,Iref=3262,Izero=2095,Vset=2291,Vset1=2340,I1A=2669,Im1A=1157,Im2A=646;
//3250  
uint32_t Itime=0;
 uint8_t Run=0;//overload=0;
int32_t CurrentDutyA,CurrentDutyA1,CurrentDutyA2,VFeed,IFeed,VFeedB,IFeedB,tempFeed;
int32_t CTMin;
int32_t CTMax;
extern int32_t Kp,Ki,Ki1,Kp1;
char Iflag=0;
unsigned char ptr[6]={0,0,'.',0,0,'\0'};
unsigned char modep[4]={' ',' ',' ','\0'};
unsigned char ptr0[6]={0,'.',0,0,'\0'};
unsigned char ptr1[6]={0,0,'.',0,0,'\0'};
unsigned char ptr2[6]={0,'.',0,0,0,'\0'};
unsigned char ptrs[]={'V',' ',' ','\0'};
unsigned char fptr[4]={0,0,0,'\0'};
unsigned char dptr[5]={0,0,'.',0,'\0'};

//unsigned char pptr[5]={0,0,0,0,'\0'};
unsigned char mode=MI,keyin=0;//,pidtime=0;
unsigned int time1=0;
unsigned int T,temp1,keytime=0,itime=0;
extern float Iavr,Vavr,Vavr1,Icharge,IC;
HRTIM_OutputCfgTypeDef output_config;
float In=0,Iuse=2000,PC=0;
uint8_t uartr=0,send=0,psetf=0;
uint32_t keynum=0;
extern void disp4(unsigned char x,unsigned char y,uint32_t num,unsigned char prtchar[5]);


//float32_t temp;

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern HRTIM_HandleTypeDef hhrtim1;
extern UART_HandleTypeDef huart3;
void dispuart(uint32_t num)
{
	uint32_t lotemp=0;
	unsigned char prtchar[5];
	//num=num*3300/4095;
	prtchar[0]=num/1000;
	lotemp=num-(prtchar[0]*1000);
	prtchar[1]=lotemp/100;
	lotemp=lotemp-prtchar[1]*100;
	prtchar[2]=lotemp/10;
	lotemp=lotemp-prtchar[2]*10;
	prtchar[3]=lotemp;
	prtchar[0]=prtchar[0]+48;
	prtchar[1]=prtchar[1]+48;
	prtchar[2]=prtchar[2]+48;
	prtchar[3]=prtchar[3]+48;
	prtchar[4]=10;
	HAL_UART_Transmit(&huart3,prtchar,5, 20);
	
}
/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
//	Vavr=pidtime;
	Itime++;
	if(Itime>=1000)
	{
		Itime=0;
		PC=IC+PC;
	}
	//pidtime=0;
	if(keyin==1)keytime++;
	if(keytime>300){keytime=0;keyin=0;};
	if(mode==Stop){itime++;}
	if(itime>2000)
		{
		mode=MI;
		PC=0;
		itime=0;
		HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 |HRTIM_OUTPUT_TA2);
		}
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles ADC1 and ADC2 interrupts.
*/
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */
	if(__HAL_ADC_GET_FLAG(&hadc1, ADC_ISR_JEOS))
	{
	//Vtemp=V;
	V=HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);//TRG2 TIMERA PA3	高电平 COMP2 
	Vavr=(Vavr*0.99)+(((float)V)*0.01);
	//I1=HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);//PA2
	}
	if(__HAL_ADC_GET_FLAG(&hadc2, ADC_ISR_JEOS))
	{
	I=HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
	Iavr=((float)Iavr)*0.999+((float)I)*0.001;
//	if(Itemp<2200);
//	else I=Itemp;//TRG4 TIMERA PA4 低电平	 COMP4

	//V1temp=V1;
	V1=HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);//PA5
	Vavr1=(Vavr1*0.99)+(((float)V1)*0.01);
	}
  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  HAL_ADC_IRQHandler(&hadc2);
  /* USER CODE BEGIN ADC1_2_IRQn 1 */

  /* USER CODE END ADC1_2_IRQn 1 */
}

/**
* @brief This function handles USART3 global interrupt and EXTI line 28 interrupt.
*/
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */
switch (uartr)
	{
		case 1: keynum=keynum*10+1; break;
		case 2: keynum=keynum*10+2; break;
		case 3: keynum=keynum*10+3; break;
		case 4: keynum=keynum*10+4; break;
		case 5: keynum=keynum*10+5; break;
		case 6: keynum=keynum*10+6; break;
		case 7: keynum=keynum*10+7; break;
		case 8: keynum=keynum*10+8; break;
		case 9: keynum=keynum*10+9; break;
		case 10: keynum=keynum*10; break;
		case 11: keynum=keynum/10;break;
		case 12: {Icharge=keynum;keynum=0;
							if(Icharge>=1000)
							{
								Iset=((Icharge-1000)/1000)*((float)(Iref-I1A))+I1A;PC=0;
							}
							else if((Icharge<1000)&&(Icharge>-1500))
							{
								Iset=((Icharge+1500)/2500)*((float)(I1A-Im1A))+Im1A;PC=0;
							}
							else if(Icharge<=-1500)
							{
								Iset=((Icharge+2300)/800)*((float)(Im1A-Im2A))+Im2A;PC=0;
							}
							break;}
						 //{Iset=Izero+keynum*(Iref-Izero)/2000;keynum=0;break;}
		case 0x20: Im1A++;Iset=Im1A;dispuart(Im1A);break;
		case 0x25: Im1A--;Iset=Im1A;dispuart(Im1A);break;
		case 0x30: Iref=keynum;Iset=keynum;keynum=0;break;
		case 0x35: Iref=keynum;Iset=keynum;keynum=0;break;
		case 0x40: Im2A++;Iset=Im2A;dispuart(Im2A);break;
		case 0x45: Im2A--;Iset=Im2A;dispuart(Im2A);break;					
		case 0x80: Iref=keynum;Iset=keynum;keynum=0;break;
		case 0x85: Izero=keynum;keynum=0;break;
		case 0x90: Iref=3285;Iset=3285;Izero=2067;break;
		case 0x95: Vset=keynum;break;
		case 0xA0: Vset=2325;break;
		case 0xA5: send=1;break;
		case 0xB0: Iref++;Iset=Iref;dispuart(Iref);break;
		case 0xB5: Iref--;Iset=Iref;dispuart(Iref);break;
		case 0xC0: I1A++;Iset=I1A;dispuart(I1A);break;
		case 0xC5: I1A--;Iset=I1A;dispuart(I1A);break;
		case 0xD0: Vset++;dispuart(Vset);break;
		case 0xD5: Vset--;dispuart(Vset);break;
		case 0xE0: Vset1++;dispuart(Vset1);break;
		case 0xE5: Vset1--;dispuart(Vset1);break;
	case 0xF0: Icharge++;dispuart(Icharge);break;
	case 0xF5: Icharge--;dispuart(Icharge);break;
		case 0xFF:{Icharge=keynum;keynum=0;
							if(Icharge>=1000)
							{
								Iset=((Icharge-1000)/1000)*((float)(Iref-I1A))+I1A;PC=0;
							}
							else if((Icharge<1000)&&(Icharge>-1500))
							{
								Iset=((Icharge+1500)/2500)*((float)(I1A-Im1A))+Im1A;PC=0;
							}
							else if(Icharge<=-1500)
							{
								Iset=((Icharge+2300)/800)*((float)(Im1A-Im2A))+Im2A;PC=0;
							}
							break;}
						 
		default: break;
	}
	if(keynum>65535)keynum=65535;
	HAL_UART_Receive_IT(&huart3,&uartr,1);
  /* USER CODE END USART3_IRQn 1 */
}

/**
* @brief This function handles EXTI line[15:10] interrupts.
*/
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

	if(keyin==0)
	{
			if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_13))
	{
			if(mode==MV){mode=MI;PC=0;HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 |HRTIM_OUTPUT_TA2);}
			//else if(mode==MI&&){mode=MV;HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 |HRTIM_OUTPUT_TA2);}
			else if((mode==MI)&&(psetf==0)){psetf=1;}
			else if((mode==MI)&&(psetf==1)){psetf=2;}
			else if((mode==MI)&&(psetf==2)){psetf=3;}
			else if((mode==MI)&&(psetf==3)){psetf=4;}
			else if((mode==MI)&&(psetf==4)){mode=MI;psetf=0;HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 |HRTIM_OUTPUT_TA2);}
			else if(mode==Stop){mode=MV;PC=0;HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 |HRTIM_OUTPUT_TA2);}

	}

		if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_14))
	{
		switch (psetf)
		{
			case 1:Iref++;Iset=Iref;break;
			case 2:I1A++;Iset=I1A;break;
			case 3:Vset++;break;
			case 4:Vset1++;break;
			default :{Icharge=Icharge+50;if(Icharge>-1500&&Icharge<0)Icharge=500;
							if(Icharge>=1000)
							{
								Iset=((Icharge-1000)/1000)*((float)(Iref-I1A))+I1A;PC=0;
							}
							else if((Icharge<1000)&&(Icharge>-1500))
							{
								Iset=((Icharge+1500)/2500)*((float)(I1A-Im1A))+Im1A;PC=0;
							}
							else if(Icharge<=-1500)
							{
								Iset=((Icharge+2300)/800)*((float)(Im1A-Im2A))+Im2A;PC=0;
							}
							break;}
		}
//		Iset=Icharge*0.5917+2071.7;
//		Iuse=Iuse+50;
//		Iset=Iuse/2000*Iref;
		//In++;
//		//Icg=In*Icharge;
		
		}
		//Iset=(Iref+In*((float)(Iref-I1A))*50/1000);
		//Iset=Iref+In*((float)(Iref-I1A))/1000*50;
	}
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_15))
	{
		
			switch (psetf)
		{
			case 1:Iref--;Iset=Iref;break;
			case 2:I1A--;Iset=I1A;break;
			case 3:Vset--;break;
			case 4:Vset1--;break;
			default :{Icharge=Icharge-50;if(Icharge<500&&Icharge>0)Icharge=-1500;
							if(Icharge>=1000)
							{
								Iset=((Icharge-1000)/1000)*((float)(Iref-I1A))+I1A;PC=0;
							}
							else if((Icharge<1000)&&(Icharge>-1500))
							{
								Iset=((Icharge+1500)/2500)*((float)(I1A-Im1A))+Im1A;PC=0;
							}
							else if(Icharge<=-1500)
							{
								Iset=((Icharge+2300)/800)*((float)(Im1A-Im2A))+Im2A;PC=0;
							}
							break;}
		}
//		Iuse=Iuse-50;
//		Iset=Iuse/2000*Iref;
		//In--;
//		Icharge=Icharge-50;
//		Iset=((Icharge-1000)/1000)*((float)(Iref-I1A))+I1A;
		//Iset=Icharge*0.5917+2071.7;
		//Iset=(Iref+In*((float)(Iref-I1A))*50/1000);
		//Ctemp=1.69*((float)Iset)-3501.1;
		//if(In<-39){Iset=Izero;In=-39;}
	}
	keyin=1;
	
	
  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */
//停止模式：在充电模式且V=1830
  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
* @brief This function handles HRTIM timer A global interrupt.
*/
void HRTIM1_TIMA_IRQHandler(void)
{
  /* USER CODE BEGIN HRTIM1_TIMA_IRQn 0 */
	if((mode==MI)&&(Vavr>(Vset1*80/100)))time1++;
	else time1=0;
	if(time1>=100)
	{
		mode=Stop;PC=0;//过冲自动停止
		HAL_HRTIM_WaveformOutputStop(&hhrtim1,HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2);
	}
	if(mode==MV){VFeed=PI_V();CurrentDutyA=VFeed;__HAL_HRTIM_SetCompare(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, VFeed);}//放电PID控制
	else if(mode==MI)	{IFeed=PI_I();CurrentDutyA=IFeed;__HAL_HRTIM_SetCompare(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, IFeed);}//充电PID控制
	
	if((Iavr>(Iref+300))&&(Vavr1<=(Vset+50))&&(Vavr1>=(Vset-50))){mode=MI;PC=0;}
		
	if(Vavr1<1816){mode=MV;PC=0;HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 |HRTIM_OUTPUT_TA2);}
	
	if (CurrentDutyA > BUCK_PWM_PERIOD / 2)	
  {
    //合理设置ADC触发时间，从而避开尖峰采样
		__HAL_HRTIM_SetCompare(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_2,DT_RISING + CurrentDutyA / 2);
		__HAL_HRTIM_SetCompare(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_4, DT_RISING + CurrentDutyA / 2);
   }
   else
   {       
		__HAL_HRTIM_SetCompare(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_2, ((BUCK_PWM_PERIOD - CurrentDutyA) / 2) + CurrentDutyA);
		__HAL_HRTIM_SetCompare(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_4, ((BUCK_PWM_PERIOD - CurrentDutyA) / 2) + CurrentDutyA);
		}
	 
//	if(mode==MV) {ptrs[0]='V';ptrs[1]=' ';modep[0]=' ';}
//	else if(mode==MI){ptrs[0]='I';ptrs[1]=' ';modep[0]=' ';}
//	else if(mode==Stop){ptrs[0]='S';ptrs[1]=' ';modep[0]=' ';}
	
  /* USER CODE END HRTIM1_TIMA_IRQn 0 */
  HAL_HRTIM_IRQHandler(&hhrtim1,HRTIM_TIMERINDEX_TIMER_A);
  /* USER CODE BEGIN HRTIM1_TIMA_IRQn 1 */

  /* USER CODE END HRTIM1_TIMA_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
