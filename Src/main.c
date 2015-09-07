/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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

/* USER CODE BEGIN Includes */
#include "oled.h"
#include "arm_math.h" 
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

HRTIM_HandleTypeDef hhrtim1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_HRTIM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
//addr:98d3:31:205353
//98d3:31:20534e

#define BUCK_PWM_PERIOD ((uint16_t) 45600) /* 120kHz 63500*/
#define SAT_LIMIT ((int32_t)(BUCK_PWM_PERIOD * 50)/100) // PI Integral term saturation value
#define MIN_DUTY_A ((int32_t)(BUCK_PWM_PERIOD * 50)/100) // % MinDuty for Buck
#define MAX_DUTY_A ((int32_t)(BUCK_PWM_PERIOD * 100)/100) // % MaxDuty for Buck

#define MI 0
#define MV 1
#define Stop 2
#define MI1 3
//#define Izero 3101
//#define Vset 2318
 void Reset_PI(void);
	
 int32_t Kp;
 int32_t Ki;
 int32_t Kp1;
 int32_t Ki1;
//static int32_t Int_term_Buck,Int_term_BuckB;
static int32_t Int_term_I,Int_term_V;
extern int32_t VoutT,VFeed,IFeed,modepdis;
extern uint32_t V,V1,I,Vrms,Irms,IoutA,IoutB,I1,Iset,Iref,Izero,Vset,Vset1,I1A,Im1A,Im2A;
extern uint32_t CTMin;
extern uint32_t CTMax;
extern int temp1;
float VSENSE,Temp;
unsigned int cont=0;
unsigned int f=50;
float D=0,Ctemp=0,IC=0,Iavr=0,Vavr=0,Vavr1=0,Icharge=2000,power=0;//Icharge=(3613-3101)*50/2000;
extern  uint8_t Run;
extern unsigned int CurrentDutyA;
extern unsigned char pptr[],ptr0[],ptr[],ptr1[],ptr2[],ptrs[],fptr[],dptr[],modep[],mode;
unsigned char Iavrp[5];
unsigned char pptr[5]={0,0,0,0,'\0'};
unsigned char pcptr[5]={0,0,'.',0,'\0'};
unsigned char ptrk[6]={0,0,0,0,0,'\0'};
unsigned char powerptr[5]={0,0,0,0,'\0'};
extern uint8_t uartr,send;
extern uint32_t keynum;
extern float PC;
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_HRTIM1_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */
	for(unsigned int i=0;i<2000;i++)
	{
		for(unsigned int j=0;j<800;j++);
	}
	OLED_Init(); 										//OLED初始化
	OLED_Fill(0x00);								//OLED清屏
//	HAL_HRTIM_WaveformOutputStart(&hhrtim1,HRTIM_OUTPUT_TA1 |HRTIM_OUTPUT_TA2);
//	HAL_HRTIM_WaveformCounterStart_IT(&hhrtim1, HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_MASTER);
	HAL_HRTIM_WaveformCounterStart_IT(&hhrtim1, HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B | HRTIM_TIMERID_MASTER);//启动定时
	HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TB1 |HRTIM_OUTPUT_TB2 |HRTIM_OUTPUT_TA1 |HRTIM_OUTPUT_TA2);//启动PWM输出
    //HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_B);
	//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15, GPIO_PIN_RESET);

	OLED_P8x16Str(0,2,"T");
	OLED_P6x8Str(55,3,"C");
	OLED_P6x8Str(57,1,"mAh");
	OLED_P8x16Str(0,4,"U");
	OLED_P8x16Str(120,4,"V");
	OLED_P6x8Str(110,5,"P");
	OLED_P6x8Str(55,5,"B");
	OLED_P8x16Str(0,6,"I");
	OLED_P8x16Str(120,6,"A");
	OLED_P6x8Str(55,7,"S");
	OLED_P6x8Str(110,7,"R");
	Reset_PI();//PI初始化
	HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);//ADC1自校正
	HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);//ADC2自校正
	HAL_ADCEx_InjectedStart_IT(&hadc1);//启动ADC1
	HAL_ADCEx_InjectedStart_IT(&hadc2);//启动ADC2
	HAL_UART_Receive_IT(&huart3,&uartr,1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	unsigned int i,j=30;
//	for(i=0;i<1000;i++)
//	{
//		for(j=0;j<800;j++);
//	}
		//HAL_UART_Transmit(&huart2,&uartr,1,100);
		HAL_ADC_Start(&hadc1);
		VSENSE=HAL_ADC_GetValue(&hadc1);
		Temp=(((1400 - VSENSE*3300/4096) / 46)*100 + 250);
		
		ptrk[0]=keynum/10000;
		temp1=keynum-(ptrk[0]*10000);
		ptrk[1]=temp1/1000;
		temp1=temp1-ptrk[1]*1000;
		ptrk[2]=temp1/100;
		temp1=temp1-ptrk[2]*100;
		ptrk[3]=temp1/10;
		temp1=temp1-ptrk[3]*10;
		ptrk[4]=temp1;
		ptrk[0]=ptrk[0]+48;
		ptrk[1]=ptrk[1]+48;
		ptrk[2]=ptrk[2]+48;
		ptrk[3]=ptrk[3]+48;
		ptrk[4]=ptrk[4]+48;
		OLED_P8x16Str(80,0,ptrk);
		
		if(Iavr<Izero){modep[0]='D';modep[1]='i';modep[2]='s';}//modep[3]='c';modep[4]='h';
			//modep[5]='a';modep[6]='r';modep[7]='g';modep[8]='e';}
		else {modep[0]='C';modep[1]='h';modep[2]='a';}//modep[6]='i';modep[7]='n';modep[8]='g';modep[3]='r';modep[4]='g';modep[5]='e';}
		if(mode==Stop){modep[0]='S';modep[1]='t';modep[2]='o';}//modep[4]='d';
			//modep[5]='b';modep[6]='y';modep[7]=' ';modep[8]=' ';}
		OLED_P8x16Str(0,0,modep);
		
		if(Iavr>2000)IC=(((float)Iavr)-I1A)*1000/(Iref-I1A)+1000;
		else IC=(((float)Iavr)-Im2A)*800/(Im1A-Im2A)-2300;

		if(IC>9999)IC=0;	
		dispI(70,6,fabs(IC),ptr0);
		
		Ctemp=Vavr*3000/Vset1;
		disp5(15,4,Ctemp,ptr1,"Vavr\n");
		if(mode==MV||mode==MI1)power=(Ctemp+IC/16-1500)/600*100;
		else if(mode==MI)power=(Ctemp-IC/16-1500)/600*100;
		if(power>100)power=100;
		if(power<=0)power=0;
		//disp4(70,2,power,pptr);
		{
//		powerptr[0]=power/100;
//			if(powerptr[0]==0)powerptr[0]=' ';
	temp1=power;
	powerptr[1]=temp1/10;
	temp1=temp1-powerptr[1]*10;
	powerptr[2]=temp1;
	//lotemp=lotemp-ptr[2]*10;
	powerptr[3]='%';
	powerptr[0]=' ';
	powerptr[1]=powerptr[1]+48;
	powerptr[2]=powerptr[2]+48;
	//ptr[3]=ptr[3]+48;
	OLED_P8x16Str(94,2,powerptr);
	}
	
		Ctemp=Vavr1*3000/Vset;
		disp5(70,4,Ctemp,ptr,"Vav1\n");
			
		
			//dispI(70,6,Iavr,ptr0);
		//Ctemp=(fabs((float)(Iset)-Izero)*2000/510);
		//Ctemp=(int)((fabs((float)(Iset)-Izero)*2000/(Iref-Izero)+0.5)*10)/10;///10+0.5*10;
			//Ctemp=1.69*((float)Iset)-3501.1;
		dispI(15,6,fabs(Icharge),ptr2);
		
		{
		fptr[0]=f/100;
		temp1=f-(fptr[0]*100);
		fptr[1]=temp1/10;
		temp1=temp1-fptr[1]*10;
		fptr[2]=temp1;
		}
		
		{
//		if(mode==MV)D= (VFeed)*100/BUCK_PWM_PERIOD;
//		else if(mode==MI)D=(IFeed)*100/BUCK_PWM_PERIOD; 
//		else if(mode==Stop)D=0;
		//disp4(25,2,Temp,dptr);
	dptr[0]=Temp/100;
	temp1=Temp-(dptr[0]*100);
	dptr[1]=temp1/10;
	temp1=temp1-dptr[1]*10;
	dptr[3]=temp1;
	//temp1=temp1-dptr[2]*10;
	//dptr[3]=temp1;
	dptr[0]=dptr[0]+48;
	dptr[1]=dptr[1]+48;
	//dptr[2]=dptr[2]+48;
	dptr[3]=dptr[3]+48;
	OLED_P8x16Str(20,2,dptr);
			if(Temp>550)mode=Stop;
		}
		//disp4(25,2,PC/3600,dptr);
		//OLED_P8x16Str(65,2,ptrs);
		
		
		disp4(25,0,PC/3600,pcptr);
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_HRTIM1|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Hrtim1ClockSelection = RCC_HRTIM1CLK_PLLCLK;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

}

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  ADC_InjectionConfTypeDef sConfigInjected;
  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC;
  hadc1.Init.Resolution = ADC_RESOLUTION12b;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = OVR_DATA_OVERWRITTEN;
  HAL_ADC_Init(&hadc1);

    /**Configure Injected Channel 
    */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_4;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_HRTIM_TRG4;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

}

/* ADC2 init function */
void MX_ADC2_Init(void)
{

  ADC_InjectionConfTypeDef sConfigInjected;

    /**Common config 
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC;
  hadc2.Init.Resolution = ADC_RESOLUTION12b;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = OVR_DATA_OVERWRITTEN;
  HAL_ADC_Init(&hadc2);

    /**Configure Injected Channel 
    */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedNbrOfConversion = 2;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_HRTIM_TRG2;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected);

    /**Configure Injected Channel 
    */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_2;
  sConfigInjected.InjectedRank = 2;
  HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected);

}

/* HRTIM1 init function */
void MX_HRTIM1_Init(void)
{

  HRTIM_ADCTriggerCfgTypeDef pADCTriggerCfg;
  HRTIM_TimeBaseCfgTypeDef pTimeBaseCfg;
  HRTIM_TimerCfgTypeDef pTimerCfg;
  HRTIM_CompareCfgTypeDef pCompareCfg;
  HRTIM_TimerEventFilteringCfgTypeDef pTimerEventFilteringCfg;
  HRTIM_OutputCfgTypeDef pOutputCfg;
  HRTIM_DeadTimeCfgTypeDef pDeadTimeCfg;

  hhrtim1.Instance = HRTIM1;
  hhrtim1.Init.HRTIMInterruptResquests = HRTIM_IT_NONE;
  hhrtim1.Init.SyncOptions = HRTIM_SYNCOPTION_NONE;
  HAL_HRTIM_Init(&hhrtim1);

  HAL_HRTIM_DLLCalibrationStart(&hhrtim1, HRTIM_CALIBRATIONRATE_14);

  HAL_HRTIM_PollForDLLCalibration(&hhrtim1, 100);

  pADCTriggerCfg.UpdateSource = HRTIM_ADCTRIGGERUPDATE_TIMER_A;
  pADCTriggerCfg.Trigger = HRTIM_ADCTRIGGEREVENT24_TIMERA_CMP2;
  HAL_HRTIM_ADCTriggerConfig(&hhrtim1, HRTIM_ADCTRIGGER_2, &pADCTriggerCfg);

  pADCTriggerCfg.Trigger = HRTIM_ADCTRIGGEREVENT24_TIMERA_CMP4;
  HAL_HRTIM_ADCTriggerConfig(&hhrtim1, HRTIM_ADCTRIGGER_4, &pADCTriggerCfg);

  pTimeBaseCfg.Period = 46000;
  pTimeBaseCfg.RepetitionCounter = 0x00;
  pTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL16;
  pTimeBaseCfg.Mode = HRTIM_MODE_CONTINUOUS;
  HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, &pTimeBaseCfg);

  pTimerCfg.InterruptRequests = HRTIM_MASTER_IT_NONE;
  pTimerCfg.DMARequests = HRTIM_MASTER_DMA_NONE;
  pTimerCfg.DMASrcAddress = 0x0;
  pTimerCfg.DMADstAddress = 0x0;
  pTimerCfg.DMASize = 0x1;
  pTimerCfg.HalfModeEnable = HRTIM_HALFMODE_DISABLED;
  pTimerCfg.StartOnSync = HRTIM_SYNCSTART_DISABLED;
  pTimerCfg.ResetOnSync = HRTIM_SYNCRESET_DISABLED;
  pTimerCfg.DACSynchro = HRTIM_DACSYNC_NONE;
  pTimerCfg.PreloadEnable = HRTIM_PRELOAD_DISABLED;
  pTimerCfg.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
  pTimerCfg.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
  pTimerCfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_DISABLED;
  HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, &pTimerCfg);

  pCompareCfg.CompareValue = 10000;
  HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, &pCompareCfg);

  pCompareCfg.CompareValue = 30000;
  HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, &pCompareCfg);

  pTimeBaseCfg.RepetitionCounter = 0;
  HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimeBaseCfg);

  pTimerCfg.InterruptRequests = HRTIM_TIM_IT_REP;
  pTimerCfg.DMARequests = HRTIM_TIM_DMA_NONE;
  pTimerCfg.DMASrcAddress = 0x0;
  pTimerCfg.DMADstAddress = 0x0;
  pTimerCfg.DMASize = 0x1;
  pTimerCfg.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
  pTimerCfg.FaultEnable = HRTIM_TIMFAULTENABLE_NONE;
  pTimerCfg.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
  pTimerCfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_ENABLED;
  pTimerCfg.DelayedProtectionMode = HRTIM_TIMDELAYEDPROTECTION_DISABLED;
  pTimerCfg.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_NONE;
  pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_MASTER_PER;
  pTimerCfg.ResetUpdate = HRTIM_TIMUPDATEONRESET_DISABLED;
  HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimerCfg);

  pTimerCfg.InterruptRequests = HRTIM_TIM_IT_NONE;
  pTimerCfg.DMASrcAddress = 0x0;
  pTimerCfg.DMADstAddress = 0x0;
  pTimerCfg.DMASize = 0x1;
  pTimerCfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_DISABLED;
  pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_NONE;
  HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pTimerCfg);

  pCompareCfg.CompareValue = 20000;
  pCompareCfg.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
  pCompareCfg.AutoDelayedTimeout = 0x0000;

  HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_2, &pCompareCfg);

  pCompareCfg.CompareValue = 15000;
  pCompareCfg.AutoDelayedTimeout = 0x0000;

  HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_2, &pCompareCfg);

  pCompareCfg.CompareValue = 20000;
  pCompareCfg.AutoDelayedTimeout = 0x0000;

  HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_4, &pCompareCfg);

  pTimerEventFilteringCfg.Filter = HRTIM_TIMEVENTFILTER_NONE;
  pTimerEventFilteringCfg.Latch = HRTIM_TIMEVENTLATCH_DISABLED;

  HAL_HRTIM_TimerEventFilteringConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_EVENT_NONE, &pTimerEventFilteringCfg);

  pOutputCfg.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_TIMPER;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_TIMCMP1;
  pOutputCfg.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
  pOutputCfg.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
  pOutputCfg.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_NONE;
  pOutputCfg.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
  pOutputCfg.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
  HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA1, &pOutputCfg);

  HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_OUTPUT_TB1, &pOutputCfg);

  pOutputCfg.SetSource = HRTIM_OUTPUTSET_NONE;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_NONE;
  HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA2, &pOutputCfg);

  HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_OUTPUT_TB2, &pOutputCfg);

  pDeadTimeCfg.Prescaler = HRTIM_TIMDEADTIME_PRESCALERRATIO_MUL8;
  pDeadTimeCfg.RisingValue = 25;
  pDeadTimeCfg.RisingSign = HRTIM_TIMDEADTIME_RISINGSIGN_POSITIVE;
  pDeadTimeCfg.RisingLock = HRTIM_TIMDEADTIME_RISINGLOCK_WRITE;
  pDeadTimeCfg.RisingSignLock = HRTIM_TIMDEADTIME_RISINGSIGNLOCK_WRITE;
  pDeadTimeCfg.FallingValue = 35;
  pDeadTimeCfg.FallingSign = HRTIM_TIMDEADTIME_FALLINGSIGN_POSITIVE;
  pDeadTimeCfg.FallingLock = HRTIM_TIMDEADTIME_FALLINGLOCK_WRITE;
  pDeadTimeCfg.FallingSignLock = HRTIM_TIMDEADTIME_FALLINGSIGNLOCK_WRITE;
  HAL_HRTIM_DeadTimeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pDeadTimeCfg);

  pDeadTimeCfg.RisingValue = 100;
  pDeadTimeCfg.FallingValue = 100;
  HAL_HRTIM_DeadTimeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pDeadTimeCfg);

  pTimeBaseCfg.RepetitionCounter = 0x00;
  HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pTimeBaseCfg);

}

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0000020B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
  HAL_I2C_Init(&hi2c1);

    /**Configure Analogue filter 
    */
  HAL_I2CEx_AnalogFilter_Config(&hi2c1, I2C_ANALOGFILTER_ENABLED);

}

/* USART3 init function */
void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 38400;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED ;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart3);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOF_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void Reset_PI(void)
{
  /* Reset integral terms for PI */
  //Int_term_Buck = 0;
	Int_term_I=0;
	Int_term_V=0;
//  Int_term_Boost = 0;
//  Int_term_Mixed = 0;
  /* Reset Counters Min and Max */
  CTMax = 0;
  CTMin = 0;
  /* Set Proportional and Integral constant terms*/
  Ki = 80;
  Kp = 100;
	Ki1=20;
	Kp1=500;
}

int32_t PI_V(void)
{
  /* Compute PI for Buck Mode */
  /* Every time the PI order sets extreme values then CTMax or CTMin are managed */
  int32_t seterr, pid_out;
  int32_t error;

  error = (int32_t ) V1 - (int32_t) Vset;
  seterr = (Kp * error);

  Int_term_V = Int_term_V + ((Ki * error));

  if (Int_term_V > SAT_LIMIT*200)
  {
    Int_term_V = SAT_LIMIT*200;
  }
  if (Int_term_V < -(SAT_LIMIT*200))
  {
    Int_term_V = -(SAT_LIMIT*200);
  }
  pid_out = seterr + Int_term_V;
  pid_out += BUCK_PWM_PERIOD *100;

  if (pid_out >= MAX_DUTY_A*200)
  {
    pid_out = MAX_DUTY_A*200;
    //CTMax++;
  }
  else
  {
    if (CTMax != 0)
    {
     // CTMax--;
    }
  }
  if (pid_out <= MIN_DUTY_A*200)
  {
    pid_out = MIN_DUTY_A*200;
    //CTMin++;
  }
  else
  {
    if (CTMin != 0)
    {
    //  CTMin--;
    }
  }
  return  pid_out/200;
}





int32_t PI_I(void)
{
  /* Compute PI for Buck Mode */
  /* Every time the PI order sets extreme values then CTMax or CTMin are managed */
  int32_t seterr, pid_out;
  int32_t error;

  error = (int32_t ) I- Iset;
  seterr = (-Kp1 * error);


  Int_term_I = Int_term_I + ((-Ki1 * error));

  if (Int_term_I > SAT_LIMIT*200)
  {
    Int_term_I = SAT_LIMIT*200;
  }
  if (Int_term_I < -(SAT_LIMIT*200))
  {
    Int_term_I = -(SAT_LIMIT*200);
  }
  pid_out = seterr + Int_term_I;
  pid_out += SAT_LIMIT*200;

  if (pid_out >= MAX_DUTY_A*200)
  {
    pid_out = MAX_DUTY_A*200;
    //CTMax++;
  }
  else
  {
    if (CTMax != 0)
    {
     // CTMax--;
    }
  }
  if (pid_out <= MIN_DUTY_A*200)
  {
    pid_out = MIN_DUTY_A*200;
    //CTMin++;
  }
  else
  {
    if (CTMin != 0)
    {
    //  CTMin--;
    }
  }
  return  pid_out / 200;
}

int32_t PI_I2(void)
{
  /* Compute PI for Buck Mode */
  /* Every time the PI order sets extreme values then CTMax or CTMin are managed */
  int32_t seterr, pid_out;
  int32_t error;

  error = (int32_t ) V1 - (int32_t) Vset*0.8;
  seterr = (Kp * error);

  Int_term_V = Int_term_V + ((Ki * error));

  if (Int_term_V > SAT_LIMIT*200)
  {
    Int_term_V = SAT_LIMIT*200;
  }
  if (Int_term_V < -(SAT_LIMIT*200))
  {
    Int_term_V = -(SAT_LIMIT*200);
  }
  pid_out = seterr + Int_term_V;
  pid_out += BUCK_PWM_PERIOD *100;

  if (pid_out >= MAX_DUTY_A*200)
  {
    pid_out = MAX_DUTY_A*200;
    //CTMax++;
  }
  else
  {
    if (CTMax != 0)
    {
     // CTMax--;
    }
  }
  if (pid_out <= MIN_DUTY_A*200)
  {
    pid_out = MIN_DUTY_A*200;
    //CTMin++;
  }
  else
  {
    if (CTMin != 0)
    {
    //  CTMin--;
    }
  }
  return  pid_out/200;
}

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
