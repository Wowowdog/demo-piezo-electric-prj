/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
- mode selection, including FRA mode
- note that if number of NUMBER_LINEAR_COMPENSATION changes, this number in function "LinearCompensate3" needs to be changed as well
-

- mode selection, including FRA mode
- note that if number of NUMBER_LINEAR_COMPENSATION changes, this number in function "LinearCompensate3" needs to be changed as well
- my_RxBuf[0] = 0x00: CLM model, target automatically changed
- my_RxBuf[0] = 0x01: // standby
- my_RxBuf[0] = 0x02: //  CLM model, target manually input
- my_RxBuf[0] = 0x03: // calibration mode for adc data input and standby
- my_RxBuf[0] = 0x04: // calibration mode for displacement data input and standby
- my_RxBuf[0] = 0x05: // OLM driving
- my_RxBuf[0] = 0x06: // FRA test
- my_RxBuf[0] = 0x07: // FRA result transmit
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"
#include <complex.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


#include "led.h"
#include "key.h"
#include "delay.h"
#include "myiic.h"
#include "usbd_cdc_if.h"
#include "functions.h"
#include "current.h"

#define NUMBER_LINEAR_COMPENSATION 11
#define NUMBER_ADC_CHANNEL 1
#define NUMBER_ADC_CHANNEL_AVERAGE_PER_CHANNEL 1 //set smaller for FRA test to alleviate calculation phase delay
#define NUMBER_FRA_TEST 30
#define NUMBER_FRA_ADC_BUFFERSIZE 1000;
#define FREQ_START           1;  // FRA start freq (Hz)
#define FREQ_STOP            100;// FRA stop freq (Hz)
#define FREQ_PONT            20; // FRA freq number

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

uint16_t ADC_DMA_BUFF[NUMBER_ADC_CHANNEL * NUMBER_ADC_CHANNEL_AVERAGE_PER_CHANNEL]={0};

uint8_t OLM_CMD_flag=0       ;
uint8_t FRA_TEST_flag=0 		 ;
uint32_t  CNT1=   0          ;
double    diff=0             ;
uint8_t   zeroCrossFlag=0    ;
uint32_t  ADC_value=   0     ;
uint32_t  ADC_value_temp=0   ;
uint32_t  timer              ;
double    timer_inSec =0     ;
uint32_t  timer_ini          ;
uint32_t  ADCvalue_filter    ; 
double    setpoint           ;  
double    setpoint_pre       ;  
double    setpoint_cmd       ;  
double    setpoint_fra       ;  
uint16_t  pulse              ; 
int8_t    signOferror =   0  ;
int8_t    signOferror_pre = 0;
u8        cData[2]={0} ;       // buf for hex2bin convert
char      buf[800];/*for UART trasmit*/

typedef struct {
		uint8_t freq_idx;
		uint8_t freq_pts;
		double  freq_start;
		double  freq_stop;
		uint16_t ADC_buff_size;
		uint16_t ADC_samp_freq;  // sampling frequency, or the interrupt frequency for ADC sampling
		double ADC_samp_wnum;  // sampling wave number, can be used to calculate/update ADC samping size
		uint16_t ADC_samp_size;
		uint16_t ADC_Nsamp_size; // the size not taking into ADC_buff. to reduce transient effect
		uint32_t ADC_buff[1000];
		double setpoint_buff[1000];
		float  freqHz[20];
//		double complex Xw[10];     // there is trouble assigning value to complex array, so after calculation, use creal() and cimag() to store result
//		double complex Yw[10];	   // so not declaring complex array
//		double complex Gw[10];
		double  Xw_Real[20];
		double  Xw_Imag[20];
		double  Yw_Real[20];
		double  Yw_Imag[20];
		double  Gw_Real[20];
		double  Gw_Imag[20];
		double phase_deg[20];
		double phase_rad[20];
		double mag[20];
} FRA_DATA;

FRA_DATA *FRA;

void ini_FRA(FRA_DATA *ptr){
		ptr->freq_idx   = 0;
		ptr->freq_pts   = 20;
		ptr->freq_start = 1;
		ptr->freq_stop  = 200;
		ptr->ADC_buff_size   = 2000;
	  ptr->ADC_samp_freq = 1000;  // depends on TIM7 interrupt even rate 
		ptr->ADC_samp_wnum = 1;     // depends on TIM7 interrupt even rate 
		ptr->ADC_samp_size = 1000;
		ptr->ADC_Nsamp_size = 400;
		FraFreq(ptr->freqHz,ptr->freq_start,ptr->freq_stop,ptr->freq_pts,2);
}

//void init_FRA_data(FRA_DATA *fra_ptr, uint16_t buff_size, uint8_t freq_number)
//{
//	fra_ptr->freq_idx = 0;
//	fra_ptr->freq_pts= freq_number;
//	fra_ptr->ADC_buff_size = buff_size;
//	fra_ptr->ADC_buff = (uint32_t*)malloc(sizeof(uint32_t*)*buff_size );
//	fra_ptr->setpoint_buff = (double*)malloc(sizeof(double*)*buff_size );
//	fra_ptr->freqHz   = (float*)malloc(sizeof(float*)*freq_number );
//	fra_ptr->Xw       = (double complex*)malloc(sizeof(double complex*)*freq_number);
//	fra_ptr->Yw       = (double complex*)malloc(sizeof(double complex*)*freq_number);
//	fra_ptr->Gw       = (double complex*)malloc(sizeof(double complex*)*freq_number);
////	fra_ptr->Xw_Real  = (double *)malloc(sizeof(double)*freq_number);
////	fra_ptr->Xw_Imag  = (double *)malloc(sizeof(double)*freq_number);
////	fra_ptr->Yw_Real  = (double *)malloc(sizeof(double)*freq_number);
////	fra_ptr->Yw_Imag  = (double *)malloc(sizeof(double)*freq_number);
//	fra_ptr->Gw_Real  = (double*)malloc(sizeof(double*)*freq_number);
//	fra_ptr->Gw_Imag  = (double*)malloc(sizeof(double*)*freq_number);

//	fra_ptr->phase    = (double*)malloc(sizeof(double*)*freq_number);
//	fra_ptr->mag      = (double*)malloc(sizeof(double*)*freq_number);
//	
//	FraFreq(fra_ptr->freqHz,1,100,freq_number,2);
//	
//	//if (fra_ptr->Xw == NULL)while(1);
//}

u8 sendData[100];

uint8_t cmd_Buf[]= {
//		0x00,0x00,
//  //  0x00,0xC8, //200 each
//		0x01,0x90,
//	//	0x02,0x58,
		0x03,0x20,
	//	0x03,0xE8,
		0x04,0xB0,
	//	0x05,0x78,
		0x06,0x40,
//		0x07,0x08,
		0x07,0xD0,
	//	0x08,0x98,
		0x09,0x60,
	//	0x0A,0x28,
		0x0A,0xF0,
	//	0x0B,0xB8,
//		0x0C,0x80,
////		0x0D,0x48,
//		0x0E,0x10,
////		0x0E,0xD8,
//	  0x0F,0xA0, //4000
//		0x10,0x68, //4200
//		0x11,0x30 //4400
	//	0x11,0xf8  //4600
};

uint8_t disp_Buf[]= {
		0x00,0x00,
//    0x00,0xC8, //200 each
		0x01,0x90,
//		0x02,0x58,
		0x03,0x20,
//		0x03,0xE8,
		0x04,0xB0,
//		0x05,0x78,
		0x06,0x40,
//		0x07,0x08,
		0x07,0xD0,
//		0x08,0x98,
		0x09,0x60,
//		0x0A,0x28,
		0x0A,0xF0,
//		0x0B,0xB8,
		0x0C,0x80,
//		0x0D,0x48,
		0x0E,0x10,
//		0x0E,0xD8,
    0x0F,0xA0, //4000
};

uint8_t adc_Buf[]= {
0x27,0x10,
//0x2F,0xDA,
0x38,0xA4,
//    0x41,0x6E,
    0x4A,0x38,
//    0x53,0x02,
    0x5B,0xCC,
//    0x64,0x96,
    0x6D,0x60,
 //   0x76,0x2A,
    0x7E,0xF4,
//    0x87,0xBE,
    0x90,0x88,
//    0x99,0x52,
    0xA2,0x1C,
//    0xAA,0xE6,
    0xB3,0xB0,
//    0xBC,0x7A,
    0xC5,0x44,
//    0xCE,0x0E,
    0xD6,0xD8,
};

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
void RecieEven(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void convertUnsignedShort2u8(unsigned short nValue, u8* cData)
{
	cData[0] = (nValue >> 8) & 0xff;
	cData[1] = nValue & 0xff;
}

uint32_t ADC_DMA_AVERAGE(int channel)
{
	uint32_t adc_sum;
	int i;
	
	adc_sum = 0;
	if(channel < NUMBER_ADC_CHANNEL )
	{
		for(i=0; i<NUMBER_ADC_CHANNEL_AVERAGE_PER_CHANNEL; i++)
			adc_sum += ADC_DMA_BUFF[channel+i*NUMBER_ADC_CHANNEL];
	}
	else
		return 1;
	
	return adc_sum/NUMBER_ADC_CHANNEL_AVERAGE_PER_CHANNEL;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		 if (htim == &htim6)
		 {
//			TIM3->CNT =0;
//			TIM4->CNT =0;
//			static int8_t toggleFlag=-1;
//	    toggleFlag = -1*toggleFlag;
//				if (toggleFlag==-1){
//		 TIM1->CCR1 = 2001;
//		__HAL_TIM_SET_COUNTER(&htim3,500);
//		__HAL_TIM_SET_COUNTER(&htim4,0);
//				}
//					else{
//					 TIM1->CCR1 = 2001;
//		__HAL_TIM_SET_COUNTER(&htim4,500);
//		 __HAL_TIM_SET_COUNTER(&htim3,0);}		
					

		
	  //ADCvalue_filter = (uint32_t) K_Filter(ADC_value, 0, 1, -0.0050171, 1e-4, 31, 2e4, 0.3);
		//uk = pulse*signOferror;
//		float diff = fabs(setpoint_cmd - ADCvalue_filter);
//		signOferror    = (setpoint_cmd - ADCvalue_filter)>0?1:-1;
//		if (diff<=100){pulse = 0;}
//		else {pulse = 4000;}	 

		static int k=0;

		setpoint = (double)((cmd_Buf[k]<<8) +cmd_Buf[k+1]);
//		my_RxBuf[0]=cmd_Buf[k];
//		my_RxBuf[1]=cmd_Buf[k+1];
		k+=2;
		k = (k+2>(sizeof(cmd_Buf)/sizeof(cmd_Buf[0])))?0:k;
	}
		 if (htim == &htim1 )
			 {
//			TIM3->CNT =0;
//			TIM4->CNT =0;
//			static int8_t toggleFlag=-1;
//	    toggleFlag = -1*toggleFlag;
//				if (toggleFlag==-1){
//		 TIM1->CCR1 = 4000;
//		__HAL_TIM_SET_COUNTER(&htim3,500);
//		__HAL_TIM_SET_COUNTER(&htim4,0);
//				}
//					else{
//					 TIM1->CCR1 = 4000;
//		__HAL_TIM_SET_COUNTER(&htim4,500);
//		 __HAL_TIM_SET_COUNTER(&htim3,0);}		
//					}
//	 
//			 TIM3->CNT =0;
//			 TIM4->CNT =0;
			 TIM1->CNT =0;
		//	 TIM7->CNT =0;
			// TIM1->CCR1 = pulse; // follows: TIM->CCR1 mod TIM3->AAR = 0
				 
			 TIM3->CNT =0;
			 TIM4->CNT =0;
			// TIM1->CNT =0;
		//	 TIM7->CNT =0;
			 TIM1->CCR1 = pulse; // follows: TIM->CCR1 mod TIM3->AAR = 0

		__HAL_TIM_SET_COUNTER(&htim4,250-250*signOferror);
		__HAL_TIM_SET_COUNTER(&htim3,250+250*signOferror);
		
		CNT1+=1;
		 }
			 
		if (htim == &htim7 && my_RxBuf[0]==0x06 )
			{
			static uint16_t counter;
			static double complex c1,c2;
				
			if (counter ==FRA->ADC_samp_size + FRA->ADC_Nsamp_size){   // verify if all ADC data stored, and do the computation if yes
				//compute complex response
					c1=0;c2=0;
					for (int i=0;i<FRA->ADC_samp_size;i++){
//					FRA->Yw[FRA->freq_idx] += FRA->ADC_buff[i]     *cexp(-1*I*2*M_PI* FRA->freqHz[FRA->freq_idx]/FRA->ADC_buff_size*i);
//					FRA->Xw[FRA->freq_idx] += FRA->setpoint_buff[i]*cexp(-1*I*2*M_PI* FRA->freqHz[FRA->freq_idx]/FRA->ADC_buff_size*i);
						c1+= FRA->ADC_buff[i]     *cexp(-1*I*2*M_PI* FRA->freqHz[FRA->freq_idx]/FRA->ADC_samp_size*i);
						c2+= FRA->setpoint_buff[i]*cexp(-1*I*2*M_PI* FRA->freqHz[FRA->freq_idx]/FRA->ADC_samp_size*i);
					}
					
//				FRA->Gw[FRA->freq_idx] = FRA->Yw[FRA->freq_idx]/FRA->Xw[FRA->freq_idx];
				FRA->Gw_Real[FRA->freq_idx] = crealf(c1/c2);
				FRA->Gw_Imag[FRA->freq_idx] = cimagf(c1/c2);
				FRA->Yw_Real[FRA->freq_idx] = crealf(c1);
				FRA->Yw_Imag[FRA->freq_idx] = cimagf(c1);
				FRA->Xw_Real[FRA->freq_idx] = crealf(c2);
				FRA->Xw_Imag[FRA->freq_idx] = cimagf(c2);
				FRA->mag[FRA->freq_idx]     = 20*log10(cabsf(c1/c2));
				FRA->phase_rad[FRA->freq_idx]   = cargf(c1/c2);
				FRA->phase_deg[FRA->freq_idx]   = cargf(c1/c2)*180/M_PI;
				
				//reset & update frequency & update sampling/Non-sampling size
				counter = 0;
				FRA->freq_idx++;
				//FRA->ADC_samp_size  = ceilf(FRA->ADC_samp_wnum*FRA->ADC_samp_freq/FRA->freqHz[FRA->freq_idx]);
				//FRA->ADC_Nsamp_size = ceilf(0.5*FRA->ADC_samp_freq/FRA->freqHz[FRA->freq_idx]) ;
			}	
			
			if (FRA->freq_idx > FRA->freq_pts-1) //verify if all freq. were all implemented, and tranmit data if yes 
			{
//				static uint8_t Enter_flag=0;
//				if (Enter_flag){
//				while(1);        // trap if re-entering
//				}
				
				
//				//data transmit
//				for (int i=0;i<FRA->freq_pts;i++){
//					sprintf(sendData, "%f,%lf,%lf\r\n", FRA->freqHz[i], FRA->Gw_Real[i],FRA->Gw_Imag[i]);
//					CDC_Transmit_HS(sendData,strlen(sendData));		
//				}
				my_RxBuf[0]=0x07; // go to standby mode
				
//				Enter_flag=1;
			}
			
			else if (counter>=FRA->ADC_Nsamp_size)
			{
				//collect data
				FRA->ADC_buff[counter-FRA->ADC_Nsamp_size] = ADC_value;
				FRA->setpoint_buff[counter-FRA->ADC_Nsamp_size] =setpoint_cmd;
			}
			
			counter++;
			}

}	

/* MPU Configuration */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_DAC1_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
	KEY_Init();
	IIC_Init();
	CURR_Init();
	LED_Init();
//	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);  /*PWM initial is included in set_phase()*/
//	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
//	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
//	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
//	u8 send_buf[2]={0x00,0x00};
//	u8 read_buf[20];
//	IIC_Write_Arr(1,0x1c,0x02,0x01,send_buf);HAL_Delay(5);
//	send_buf[0]=0x3b;IIC_Write_Arr(1,0x1c,0xae,0x01,send_buf);HAL_Delay(5);	
//	send_buf[0]=0x7b;IIC_Write_Arr(1,0x1c,0xa6,0x01,send_buf);HAL_Delay(5);	
//	send_buf[0]=0x00;IIC_Write_Arr(1,0x1c,0x02,0x01,send_buf);HAL_Delay(5);
//	u8 curr[2]={0x03,0xFF};
//	IIC_Write_Arr(8,0x18,0x03,2,curr);

	HAL_DAC_Start(&hdac1,DAC_CHANNEL_1);	
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2048);

//  my_RxBuf[0]=0x07;
//	my_RxBuf[1]=0xd0;
		my_RxBuf[0]=0x06 ;
	
//	FraFreq(freqHz,freq_start,freq_stop,FREQPTS,2);
//	pwm_ini();
	u8 volt_piezo[3]={0x20,0x4e,0x01}; 
	//u8 volt_piezo[3]={0x66,0x66,0x01}; 
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_11,GPIO_PIN_SET);
	IIC_Write_Arr(8,0xc4,0x00,3,volt_piezo);
	IIC_Write_Arr(9,0xc4,0x00,3,volt_piezo);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);

	HAL_ADCEx_Calibration_Start(&hadc1,ADC_CALIB_OFFSET_LINEARITY,ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *) ADC_DMA_BUFF, NUMBER_ADC_CHANNEL * NUMBER_ADC_CHANNEL_AVERAGE_PER_CHANNEL);
	
	ADC_value_temp = ADC_DMA_AVERAGE(0);
	//init_FRA_data(FRA, 1000, 10);
	ini_FRA(FRA);
	
	__HAL_TIM_SET_COUNTER(&htim3,0);
	__HAL_TIM_SET_COUNTER(&htim4,0);
	
	//	HAL_TIM_Base_Start_IT(&htim7);
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);
	

	/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		/*timer*/
//			timer = GetMicrosFromISR_Reset(&timer_ini);
//		  timer_inSec = (float)timer/1000000;
		 #if TESTMODE==9
		  ADC_value=ADC_DMA_AVERAGE(0);
			signOferror=1;	 
			ADCvalue_filter = (uint32_t) K_Filter(ADC_value, 0, 1, -0.0050171, 1e-5, 31, 2e4, 0.3);
			//ADCvalue_filter  = iirFilter(ADC_value);
		//control 
//		 int k = floor(timer_inSec/5);
//	   setpoint = (k%2==0)?10000:50000;
		
//		setpoint = 2000*sin(2*3.1416*4*timer_inSec)+33000;
//		
//	  float diff = fabs(setpoint - ADC_value);
//		signOferror    = (setpoint - ADC_value)>0?1:-1;
//		if (diff<=50){pulse = 0;}
//		else {pulse = 2000;}
     #elif TESTMODE==8
//		//HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);
//		ADC_value=ADC_DMA_AVERAGE(0);
//	  ADCvalue_filter = (uint32_t) K_Filter(ADC_value, 0, 1, -0.0050171, 1e-5, 31, 2e4, 0.3);
//    //control 
//		setpoint = (double)((my_RxBuf[0]<<8) +my_RxBuf[1]);
//		setpoint_cmd = LinearCompensate2(setpoint);

//		ADCvalue_ctrl = ADC_value;
//		//double diff = fabs(setpoint_cmd - ADCvalue_ctrl);
//		signOferror    = (setpoint_cmd - ADCvalue_ctrl)>0?1:-1;

switch (my_RxBuf[0])
{
			
			
case 0x00: // automatic target mode
		{
		static uint16_t CNT;
		static double diff;
		//if ((zeroCrossFlag==1) && (diff>100)){CNT=0;zeroCrossFlag=0;} 
		do{
					//HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);
//			ADC_value=ADC_DMA_AVERAGE(0);
//			ADCvalue_filter = (uint32_t) K_Filter(ADC_value, 0, 1, -0.0050171, 1e-5, 31, 2e4, 0.3);
			
			ADC_value_temp=ADC_DMA_AVERAGE(0);
			//ADCvalue_filter = (uint32_t) K_Filter(ADC_value_temp, 0, 1, -0.0050171, 5e-6, 31, 2e4, 0.3);
			ADCvalue_filter = (uint32_t) K_Filter(ADC_value_temp, 0, 1, -0.0050171, 1e-5, 31, 2e4, 0.3);
			//ADCvalue_filter = (uint32_t) K_Filter(ADC_value, 0, 1, -0.0050171, 1e-5, 31, 2e4, 0.3);
			ADC_value = fabs(ADCvalue_filter-setpoint_cmd)<3?ADCvalue_filter:ADC_value_temp;

			//setpoint = (double)(( [0]<<8) +my_RxBuf[1]);
			setpoint_cmd = LinearCompensate3(setpoint, disp_Buf, adc_Buf);
			//setpoint_cmd = LinearCompensate2(setpoint);
			signOferror    = (setpoint_cmd - ADC_value)>0?1:-1;
		  diff = fabs(setpoint_cmd - ADC_value);
			if (CNT!=0) pulse=0;
			CNT++;
		} while((zeroCrossFlag==1 && CNT<80 && diff<100) || diff<5); //when flag==1, it would go inside loop untill CNT>2000 unless under exception of diff>100. When precious satisfied, then check additional condition diff<5 for whether leave the loop;
		//while( zeroCrossFlag==1 && CNT<10000 && diff<100);
		
		CNT=0;
		zeroCrossFlag=0;
		
		//if (signOferror*signOferror_pre<0){zeroCrossFlag=1;pulse=0;}
		if (signOferror*signOferror_pre<0){zeroCrossFlag=1;pulse=0;}
		else if (diff>=700){pulse = 8000;}
		else if (diff>=200){pulse = 6150;}
		else if (diff>=100){pulse = 4150;}
		else if (diff>=3){pulse = 2150;}
		else {pulse = 0;}			
		
		signOferror_pre = signOferror;
		break;
		}
			
case 0x01:   // standby mode
		{
		pulse=0;
		break;
		}
case 0x02:  // manual input target mode
	{ 
		static uint16_t CNT;
		static double diff;
			do{
					//HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);
//			ADC_value=ADC_DMA_AVERAGE(0);
//			ADCvalue_filter = (uint32_t) K_Filter(ADC_value, 0, 1, -0.0050171, 1e-5, 31, 2e4, 0.3);
			
			ADC_value_temp=ADC_DMA_AVERAGE(0);
			
			ADCvalue_filter = (uint32_t) K_Filter(ADC_value_temp, 0, 1, -0.0050171, 5e-6, 31, 2e4, 0.3);
			//ADCvalue_filter = (uint32_t) K_Filter(ADC_value, 0, 1, -0.0050171, 1e-5, 31, 2e4, 0.3);
			ADC_value = fabs(ADCvalue_filter-setpoint_cmd)<3?ADCvalue_filter:ADC_value_temp;

			setpoint = (double)((my_RxBuf[1]<<8) +my_RxBuf[2]);
			setpoint_cmd = LinearCompensate3(setpoint, disp_Buf, adc_Buf);
			//setpoint_cmd = LinearCompensate2(setpoint);
			signOferror    = (setpoint_cmd - ADC_value)>0?1:-1;
		  diff = fabs(setpoint_cmd - ADC_value);
			if (CNT!=0) pulse=0;
			CNT++;
		} while((zeroCrossFlag==1 && CNT<80 && diff<100) || diff<5); //when flag==1, it would go inside loop untill CNT>2000 unless under exception of diff>100. When precious satisfied, then check additional condition diff<5 for whether leave the loop;
		//while( zeroCrossFlag==1 && CNT<10000 && diff<100);
		
		CNT=0;
		zeroCrossFlag=0;
		
		//if (signOferror*signOferror_pre<0){zeroCrossFlag=1;pulse=0;}
		if (signOferror*signOferror_pre<0){zeroCrossFlag=1;pulse=0;}
		else if (diff>=700){pulse = 8000;}
		else if (diff>=200){pulse = 6150;}
		else if (diff>=100){pulse = 4150;}
		else if (diff>=3){pulse = 2150;}
		else {pulse = 0;}	
		
		signOferror_pre = signOferror;
		break;
		}
case 0x03: // calibration for adc and standby
		{
			for (int i=0; i<2*NUMBER_LINEAR_COMPENSATION; i++)
				{
					adc_Buf[i]  = my_RxBuf[i+1];
					//adc_Buf[i]  = 0;
				}
		pulse=0;
		while(my_RxBuf[0]==0x03);
		break;
		}		
		
case 0x04: // calibration for displacement and standby
		{
		for (int i=0; i<2*NUMBER_LINEAR_COMPENSATION; i++)
			{
				disp_Buf[i]  = my_RxBuf[i+1];
			}
		pulse=0;
		while(my_RxBuf[0]==0x04);
		break;
		}	
		
case 0x05: // open loop wave ctrl
		{			
		ADC_value_temp=ADC_DMA_AVERAGE(0);
		//ADCvalue_filter = (uint32_t) K_Filter(ADC_value_temp, 0, 1, -0.0050171, 3e-5, 31, 2e4, 0.3);
		ADCvalue_filter = (uint32_t) K_Filter(ADC_value_temp, 0, 1, -0.0050171, 5e-6, 31, 2e4, 0.3);
		ADC_value = fabs(ADCvalue_filter-setpoint_cmd)<3?ADCvalue_filter:ADC_value_temp;
 
		RecieEven();
		if (OLM_CMD_flag==1) // flag setup here allows run once per Rx receiving
			{
		signOferror = (my_RxBuf[1]==0)?1:-1;
		CNT1=0;
		pulse = 8000;
		while(CNT1<((my_RxBuf[2]<<8)+my_RxBuf[3]));
	  pulse = 0;
		OLM_CMD_flag=0;
			}
		break;	
		}

case 0x06: // FRA test 
	{
		static uint16_t CNT;
		static double diff;
		static float freq;
		timer = GetMicrosFromISR_Reset(&timer_ini);
		timer_inSec = GetSecFromISR_Reset(&timer);	
		RecieEven();
		//if ((zeroCrossFlag==1) && (diff>100)){CNT=0;zeroCrossFlag=0;} 
//		if (FRA_TEST_flag==1)
//		{
		//for (int i=0;i<(sizeof(freqHz)/sizeof(freqHz[0]));i++)
		
		//setpoint_fra = 40*sin(2*M_PI*freqHz[i]*timer_inSec);
		freq = FRA->freqHz[FRA->freq_idx];
		setpoint_fra = 2000+500*sin(2*M_PI*freq*timer_inSec);
		//ADCvalue_filter = (uint32_t) K_Filter(ADC_value, 0, 1, -0.0050171, 1e-5, 31, 2e4, 0.3);
			
//		do{
//					//HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);
//			ADC_value=ADC_DMA_AVERAGE(0);
//			ADCvalue_filter = (uint32_t) K_Filter(ADC_value, 0, 1, -0.0050171, 1e-5, 31, 2e4, 0.3);
//			
//			ADC_value=ADC_DMA_AVERAGE(0);
//			//ADCvalue_filter = (uint32_t) K_Filter(ADC_value_temp, 0, 1, -0.0050171, 5e-6, 31, 2e4, 0.3);
//			//ADCvalue_filter = (uint32_t) K_Filter(ADC_value_temp, 0, 1, -0.0050171, 1e-5, 31, 2e4, 0.3);
//			//ADCvalue_filter = (uint32_t) K_Filter(ADC_value, 0, 1, -0.0050171, 1e-5, 31, 2e4, 0.3);
//			//ADC_value = fabs(ADCvalue_filter-setpoint_cmd)<3?ADCvalue_filter:ADC_value_temp;

//			//setpoint = (double)(( [0]<<8) +my_RxBuf[1]);
//			setpoint_cmd = LinearCompensate3(setpoint_fra, disp_Buf, adc_Buf);
//			//setpoint_cmd = LinearCompensate2(setpoint);
//			signOferror    = (setpoint_cmd - ADC_value)>0?1:-1;
//		  diff = fabs(setpoint_cmd - ADC_value);
//			if (CNT!=0) pulse=0;
//			CNT++;
//		} while((zeroCrossFlag==1 && CNT<80 && diff<100) || diff<5); //when flag==1, it would go inside loop untill CNT>2000 unless under exception of diff>100. When precious satisfied, then check additional condition diff<5 for whether leave the loop;
//		//while( zeroCrossFlag==1 && CNT<10000 && diff<100);
//		
//		CNT=0;
//		zeroCrossFlag=0;
		
		ADC_value=ADC_DMA_AVERAGE(0);
		setpoint_cmd = LinearCompensate3(setpoint_fra, disp_Buf, adc_Buf);
		//setpoint_cmd = LinearCompensate2(setpoint);
		signOferror    = (setpoint_cmd - ADC_value)>0?1:-1;
		diff = fabs(setpoint_cmd - ADC_value);
		
//		//if (signOferror*signOferror_pre<0){zeroCrossFlag=1;pulse=0;}
//		if (signOferror*signOferror_pre<0){zeroCrossFlag=1;pulse=0;}
//		else 
		if (diff>=700){pulse = 8000;}
		else if (diff>=200){pulse = 6150;}
		else if (diff>=100){pulse = 4150;}
		else if (diff>=3){pulse = 0;}
		else {pulse = 0;}				
		
		//signOferror_pre = signOferror;
		
		//FRA_TEST_flag=0;
		
//		}
		break;
		}
	
case 0x07: // send FRA data
{
				//data transmit
		for (int i=0;i<FRA->freq_pts;i++){
			sprintf(sendData, "%f,%lf,%lf\r\n", FRA->freqHz[i], FRA->mag[i],FRA->phase_deg[i]);
			CDC_Transmit_HS(sendData,strlen(sendData));		
		}
		my_RxBuf[0]=0x01;
		break;
}	
}

		#endif
		 
		 #if UARTMODE==1
				sprintf(buf, "%lu,%d,%d,%d,%d\r\n", timer, (int)pulse,ADC_value,(int)ADCvalue_filter,k_stamp);
				//sprintf (buf, "%lu,%d,%s,%d\r\n", timer, (int)duty,buf_float_1,k_stamp);
				HAL_UART_Transmit(&huart2,(uint8_t*)&buf,strlen(buf),100);
		 #endif
		 
		  #if USBTRANSFER==1
			/*send real time data*/
			//float2str(buf_float_1,&Curr);
		  //sprintf (sendData, "%lu,%d,%d,%d\r\n", timer, ADC_value,ADCvalue_filter, uk);
			//sprintf (sendData, "%lu,%s,%d\r\n", timer, buf_float_1,ADC_value);
			//CDC_Transmit_HS(sendData,strlen(sendData));	 //do it in interrut
		 	#endif

//			if(recive_flag==1)
//			switch(my_RxBuf[2])
//			{
//				case 0x33: 
//					{
//						while(my_RxBuf[2]==0x33){stop();};recive_flag = 0; ;
//					}
//					{
//						while(my_RxBuf[2]==0x34){};recive_flag = 0;
//					}
//				default:break;
//			switch(my_RxBuf[2])
//			{
//				case 0x04:IIC_Write_Arr(8,0X18,0X03,2,my_RxBuf+3);break;
//				case 0x05:IIC_Write_Arr(9,0X18,0X03,2,my_RxBuf+3);break;
//				case 0x06:ad_conv();break;
//				case 0x07:ad_conv();break;
//				
//				case 0x11:IIC_Write();break;
//				case 0x12:IIC_Read();break;
//				case 0x13:IICboardcast();break;
//				case 0x14:CL_AF_CURR();break;
//				case 0x15:CL_OIS_CURR();break;
//				case 0x16:IIC_Read_GT24(0X1212,1);CDC_Transmit_HS(my_iic,1);break;
//				case 0x17:balltest();
//				case 0x19:sinroute();
////				case 0x20:maotouyinshouming();
//				case 0x21:oisrealtime_curr_x();
//				case 0x22:oisrealtime_curr_y();
//				case 0x23:oissin_curr_x();
//				case 0x24:oissin_curr_y();
//				case 0x30:set_phase();break;
//			  case 0x20:start();break;
//				case 0x32:stop();break;				
//				default:break;
//			}
		

//		KEY_Scan();

  /* USER CODE END 3 */
}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 16;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void RecieEven(void)
{
switch (my_RxBuf[0])
{
case 0x05:
	{
		if(recive_flag==1)
				{
	//	sprintf(sendData, "%lu,%d,%hhu\r\n", timer, ADC_value,recive_flag);
	//  CDC_Transmit_HS(sendData,strlen(sendData));	
			OLM_CMD_flag = 1;
			recive_flag=0;
				}
				break;
	}
	
case 0x06:
	{
		if(recive_flag==1)
		{
//			FRA_TEST_flag = 1;
			ini_FRA(FRA);
			recive_flag=0;
		}
		break;
	}
}
}

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
