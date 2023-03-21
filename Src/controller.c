#include "main.h"
#include "functions.h"

///*controller*/
float sensed_output  =0;
float control_signal =0;
float control_signal_discrete[2] ={0,0};

float last_sensed    =0;
float last_time_inSec=0;
float error;
float error_array[3]  ;
float error_array_filter[3];
float total_error;
float last_error;
const float max_control    = 10; // not 90 otherwize there's speed dead zone
const float min_control    =-10; // 

int   k_pre           ;
float t_pre           ;

/*filter*/
#if FILPARAM ==1             // order 2, Fs = 100, dB = -80
	float    bk[4]        ={1,	3,	3, 1};
	float    ak[3]        ={2.989444260935312,	2.978944160175574,0.989499752992802};
	float    inputBuf[4];
	float    outputBuf[3];
	float    inputBuf_2[4];
	float    outputBuf_2[3];
	size_t   nb           =4;
	size_t   na           =3;



#elif FILPARAM ==2           // chebyshev 2, order 2,Fstop 100Hz, dB = -80, ts = 350us
	double    bk[6]        ={1	,5,	10	,10	,5	,1};
	double    ak[5]        ={-4.99430681207885,	9.97724345114442,	-9.96588945246791,	4.97727579984891,	-0.994322986446549};
	double    inputBuf[6];
	double    outputBuf[5];
	float    inputBuf_2[6];
	float    outputBuf_2[5];
	size_t   nb           =6;
	size_t   na           =5;


#elif FILPARAM ==3           // order 10, Fpass = 20Hz, Fstop 25Hz, dB = -40
	float    bk[4]        ={0.0018187,-0.0017805,-0.0017805,0.0018187};
	float    ak[3]        ={-2.9151,2.8338,-0.91862};
	float    inputBuf[4];
	float    outputBuf[3];
	size_t   nb           =4;
	size_t   na           =3;



#elif FILPARAM ==4           // order 1, Fpass = 20Hz, Fstop 25Hz, dB = -40
	float    bk[3]        ={0.015503910683361,0.027570087324856,0.015503910683361};
	float    ak[2]        ={-1.686004274144930,1.686004274144930};
	float    inputBuf[3];
	float    outputBuf[2];
	size_t   nb           =3;
	size_t   na           =2;
#elif FILPARAM ==5           // order 10, Fpass = 20Hz, Fstop 25Hz, dB = -40
	float    bk[2]        ={0.366025403784439,0.366025403784439};
	float    ak[1]        ={0.267949192431123};
	float    inputBuf[2];
	float    outputBuf[1];
	size_t   nb           =2;
	size_t   na           =1;
#elif FILPARAM ==6           // fir filter order 5
	float    bk[6]        ={0.250024182339575,	-1.07691633388640e-14,	1.36002320516582e-14,	1.36002320516582e-14,	-1.07691633388640e-14,	0.250024182339575};
	float    ak[5]        ={0,0,0,0,0};
	float    inputBuf[6];
	float    outputBuf[5];
	size_t   nb           =6;
	size_t   na           =0;
#endif

#if PIDPARAM ==1             // piezo, stable for pulse amp. control
	float Kp          = 0.01;
	float Ki          = 0.1;           
#elif PIDPARAM ==2           // piezo, my first table phase control
	float Kp        = 0.00001;
	float Ki        = 0.0001;
#elif PIDPARAM ==3           // piezo, unstable
	float Kp        = 0.00001;
	float Ki        = 0.00001;
	float Kd        = 0.00000001;
	
#elif PIDPARAM ==5           // piezo, unstable
	float Kp        = 0.0836;
	float Ki        = 0.983 ;
	float Kd        = 0.00179;

#elif PIDPARAM ==6           // pizeo, kpkd>ki
	float Kp        = 0.1;
	float Ki        = 0.001;
	float Kd        = 0.01;
#endif
/******************************************************************************
* Function Name: iirFilter
* Description : shift signal and implement iir filter with defined coef.
* Arguments : input signal, input and output buffer, coefficients and sizes
* Return Value : filtered signal
******************************************************************************/
uint32_t iirFilter(uint32_t input){
	float sum_yn=0,sum_xn=0, res=0;
	//delay x[n]
	for (int i=0; i<nb; i+=1){
//		i == nb ? inputBuf[nb-i]=input : inputBuf[nb-i]=inputBuf[nb-1-i];
		if (i == nb-1){
			inputBuf[0]=input;
		}
		else {
			inputBuf[nb-i-1]=inputBuf[nb-i-2];
		}
	}

	//y[n] summation
	for (int i=0; i<na; i+=1){
//		float y_ = -1*ak[i]*outputBuf[i];
		float y_ = -1*ak[i]*outputBuf[i];
		sum_yn=sum_yn+y_;
	}

	//x[n] summation
	for (int i=0; i<nb; i+=1){
//		float x_ = bk[i]*inputBuf[i];
		float x_ = bk[i]*inputBuf[i];
		sum_xn=sum_xn+x_;
	}

	//%delay yn
	for (int i=0; i<na; i+=1){
//		i == na ? outputBuf[na-i]=sum_yn+sum_xn  : outputBuf[na-i]=outputBuf[na-1-i];
		if(i == na-1) {
			res = sum_yn+sum_xn;
			outputBuf[0]=res;
		}
		else {
			outputBuf[na-i-1]=outputBuf[na-i-2];
	}
  }
	return res;
}
/******************************************************************************
* Function Name: iirFilter
* Description : shift signal and implement iir filter with defined coef.
* Arguments : input signal, input and output buffer, coefficients and sizes
* Return Value : filtered signal
******************************************************************************/
uint32_t firFilter(uint32_t input){
	float sum_xn=0, res=0;
	//delay x[n]
	for (int i=0; i<nb; i+=1){
//		i == nb ? inputBuf[nb-i]=input : inputBuf[nb-i]=inputBuf[nb-1-i];
		if (i == nb-1){
			inputBuf[0]=input;
		}
		else {
			inputBuf[nb-i-1]=inputBuf[nb-i-2];
		}
	}

	//x[n] summation
	for (int i=0; i<nb; i+=1){
//		float x_ = bk[i]*inputBuf[i];
		float x_ = bk[i]*inputBuf[i];
		sum_xn=sum_xn+x_;
	}

	return sum_xn;
}
/******************************************************************************
* Function Name: iirFilter_2
* Description : use double (16 decimal precision compared to 7 of float type ), to avoid instability by truncation, since many filters have poles very close unit circle
* Arguments : input signal, input and output buffer, coefficients and sizes
* Return Value : filtered signal
******************************************************************************/
double iirFilter_2(double input, double *inputBuf, double *outputBuf, double *ak, double *bk, size_t na,size_t nb){
	double sum_yn=0,sum_xn=0, res=0;

	//delay x[n]
	for (int i=0; i<nb; i+=1){
//		i == nb ? inputBuf[nb-i]=input : inputBuf[nb-i]=inputBuf[nb-1-i];
		if (i == nb-1){
			inputBuf[0]=input;
		}
		else {
			inputBuf[nb-i-1]=inputBuf[nb-i-2];
		}
	}

	//y[n] summation
	for (int i=0; i<na; i+=1){
//		float y_ = -1*ak[i]*outputBuf[i];
		double y_ = -1*ak[i]*outputBuf[i];
		sum_yn=sum_yn+y_;
	}

	//x[n] summation
	for (int i=0; i<nb; i+=1){
//		float x_ = bk[i]*inputBuf[i];
		double x_ = bk[i]*inputBuf[i];
		sum_xn=sum_xn+x_;
	}

	//%delay yn
	for (int i=0; i<na; i+=1){
//		i == na ? outputBuf[na-i]=sum_yn+sum_xn  : outputBuf[na-i]=outputBuf[na-1-i];
		if(i == na-1) {
			res = sum_yn+sum_xn;
			outputBuf[0]=res;
		}
		else {
			outputBuf[na-i-1]=outputBuf[na-i-2];
	}
  }
	return res;
}

/******************************************************************************
* Function Name: GetMicrosFromISR
* Description : micro-second timer from ISR interrupt
* Arguments : none
* Return Value : micro-second time stamp
******************************************************************************/
uint32_t GetMicrosFromISR()
{
    uint32_t st = SysTick->VAL;
    uint32_t pending = SCB->ICSR & SCB_ICSR_PENDSTSET_Msk;
    uint32_t ms = HAL_GetTick();
    if (pending == 0)
        ms++;
    return ms * 1000 - st / ((SysTick->LOAD + 1) / 1000);
}
/******************************************************************************
* Function Name: GetMicrosFromISR_Reset
* Description : micro-second timer from ISR interrupt, start at zero initially
* Arguments : (p) timer variable, (p)initial time stamp
* Return Value : nona
******************************************************************************/
uint32_t GetMicrosFromISR_Reset(uint32_t *iniTime)
{
	return (GetMicrosFromISR()- *iniTime);
}

/******************************************************************************
* Function Name: GetSecFromISR_Reset
* Description : second timer from ISR interrupt, start at zero initially
* Arguments : (p) timer variable, (p)initial time stamp
* Return Value : nona
******************************************************************************/
float GetSecFromISR_Reset(uint32_t *timer)
{
	float timer_ = (float) *timer;
	return timer_/1000000;
}

/******************************************************************************
* Function Name: float2string
* Description : convert float point to char array buffer, for later UART
* Arguments : char array, floating value
* Return Value : none
******************************************************************************/
void float2str(char *str, float *value){
	str = (char*)malloc(sizeof(char)*80);  //pre-allocate string size
	char *valSign = (*value < 0) ? "-" : "";
	float Val     = (*value < 0) ? -(*value) : *value;

	int   Int1 = Val;                  // Get the integer
	float Frac = Val - Int1;           // Get fraction
	int   Int2 = trunc(Frac * 10000);  // Turn into integer
	sprintf(str, "%s%d.%d", valSign,Int1, Int2);
}
/******************************************************************************
* Function Name: chirp_linear
* Description : get chirp signal where freq. changes linearly with time
* Arguments : amplitude, start freq., stop freq., current time, stop time
* Return Value : chirp signal
******************************************************************************/
float chirp_linear(float amp, float f1,float f2, float time, float timer_inSec_offset, float time_max)
{
	float res;
	res = amp*sin(f1*2*3.1415926*(time-timer_inSec_offset)+(f2-f1)*2*3.1415926*(time-timer_inSec_offset)*(time-timer_inSec_offset)/time_max/2);
	return res;
}
/******************************************************************************
* Function Name: chirp_log
* Description : get chirp signal where freq. changes logarithmically w/ time
* Arguments : amplitude, start freq., stop freq., current time, stop time, (p)previous time, (p)phase
* Return Value : chirp signal
******************************************************************************/

float chirp_log(float amp, float f1,float f2, float time, float timer_inSec_offset, float time_max, float *timer_previous, float *phase)
{
	float delta_time, a0, a1, phase_increment, res;
	delta_time   =  time-timer_inSec_offset-*timer_previous;
	a0 = log10(f1*2*3.1415926);
	a1 = (log10(f2*2*3.1415926)-log10(f1*2*3.1415926))/time_max;
	phase_increment = pow(10,(a0+a1*(time-timer_inSec_offset)))*delta_time;
	phase_increment = pow(10,(a0+a1*(time-timer_inSec_offset)))*delta_time;
	*phase+=phase_increment;
	*timer_previous = time-timer_inSec_offset;
	res = amp*sin(*phase);
	return res;
}

/******************************************************************************
* Function Name: iirFilter
* Description : shift signal and implement iir filter with defined coef.
* Arguments : input signal, input and output buffer, coefficients and sizes
* Return Value : filtered signal
******************************************************************************/

float PID_Discrete(float *setpoint,float y,float adjustGain, float timer_inSec)
{
  float control_output_increment;
  float current_time = timer_inSec; //returns the number of milliseconds passed since the Arduino started running the program
  #if PIDMODE ==2
  float Ti           = Kp/Ki;
  float delta_time = current_time - last_time_inSec; //delta time interval

	#elif PIDMODE ==3
  float Ti           = Kp/Ki;
  float Td           = Kd/Kp;
  float delta_time = current_time - last_time_inSec; //delta time interval
  #endif

	sensed_output = y;  //measured PV
	float target = *setpoint;
	error_array[0] = target - sensed_output;
//    total_error += error; //accumulates the error - integral term
//    delta_error = error - last_error; //for derivative on error
//    *pdelta_error = -1*(sensed_output - last_sensed); //for derivative on PV

	 /*anti wind-up, the algorithm follows Bisoffi: Reset integral control for improved settling of PID-based motion systems with friction*/



  #if PIDMODE ==1
	control_output_increment = Kp*adjustGain*(error_array[0]-error_array[1]);
  #elif PIDMODE ==2
	control_output_increment = Kp*adjustGain*(error_array[0]-error_array[1]+error_array[0]*delta_time/Ti);
  #elif PIDMODE ==3
	control_output_increment = Kp*adjustGain*(error_array[0]-error_array[1]+error_array[0]*delta_time/Ti+Td*(error_array[0]-2*error_array[1]+error_array[2])/delta_time);
  #elif PIDMODE ==4
	error_array_filter[0] = 0.9*error_array_filter[1]+0.1*error_array[1];
	control_output_increment = Kp*adjustGain*(error_array[0]-error_array[1]+error_array[0]*delta_time/Ti+Td*(error_array_filter[0]-2*error_array_filter[1]+error_array_filter[2])/delta_time);
	control_output_increment = Kp*adjustGain*(error_array[0]-error_array[1]+error_array[0]*delta_time/Ti+Td*(error_array_filter[0]-2*error_array_filter[1]+error_array_filter[2])/delta_time);
  #endif
	control_signal_discrete[0] = control_signal_discrete[1] +control_output_increment;

	if (control_signal_discrete[0]>= max_control) control_signal_discrete[0] = max_control;
	else if (control_signal_discrete[0] <= min_control) control_signal_discrete[0] = min_control;

	error_array[2] = error_array[1];
	error_array[1] = error_array[0];
	control_signal_discrete[1] = control_signal_discrete[0];
	last_time_inSec = current_time;
  #if PIDMODE ==4
	error_array_filter[2]=error_array_filter[1];
	error_array_filter[1]=error_array_filter[0];
  #endif
	last_sensed = sensed_output; // for derivative on PV
	int8_t sign_error    = error_array[0]>0?1:(error_array[0] < 0?-1:0);
  return control_signal_discrete[0];
}
float PID_Discrete2(float *setpoint,float y,float adjustGain, float timer_inSec,float kpValue,float kpDec,float kiValue,float kiDec)
{
	Kp = kpValue/pow(10,kpDec);
	Ki = kiValue/pow(10,kiDec);
  float control_output_increment;
  float current_time = timer_inSec; //returns the number of milliseconds passed since the Arduino started running the program
  #if PIDMODE ==2
  float Ti           = Kp/Ki;
  float delta_time = current_time - last_time_inSec; //delta time interval

	#elif PIDMODE ==3
  float Ti           = Kp/Ki;
  float Td           = Kd/Kp;
  float delta_time = current_time - last_time_inSec; //delta time interval
  #endif

	sensed_output = y;  //measured PV
	float target = *setpoint;
	error_array[0] = target - sensed_output;
//    total_error += error; //accumulates the error - integral term
//    delta_error = error - last_error; //for derivative on error
//    *pdelta_error = -1*(sensed_output - last_sensed); //for derivative on PV

	 /*anti wind-up, the algorithm follows Bisoffi: Reset integral control for improved settling of PID-based motion systems with friction*/



  #if PIDMODE ==1
	control_output_increment = Kp*adjustGain*(error_array[0]-error_array[1]);
  #elif PIDMODE ==2
	control_output_increment = Kp*adjustGain*(error_array[0]-error_array[1]+error_array[0]*delta_time/Ti);
  #elif PIDMODE ==3
	control_output_increment = Kp*adjustGain*(error_array[0]-error_array[1]+error_array[0]*delta_time/Ti+Td*(error_array[0]-2*error_array[1]+error_array[2])/delta_time);
	//control_output_increment = Kp*adjustGain*(error_array[0]-error_array[1]+error_array[0]*delta_time/Ti+Td*(sensed_output-last_sensed-2*error_array[1]+error_array[2])/delta_time);
  //control_output_increment = Kp*adjustGain*(yFilter-last_sensed+error_array[0]*delta_time/Ti+Td*(sensed_output-last_sensed-2*error_array[1]+error_array[2])/delta_time);

  #elif PIDMODE ==4
	error_array_filter[0] = 0.9*error_array_filter[1]+0.1*error_array[1];
	control_output_increment = Kp*adjustGain*(error_array[0]-error_array[1]+error_array[0]*delta_time/Ti+Td*(error_array_filter[0]-2*error_array_filter[1]+error_array_filter[2])/delta_time);
	control_output_increment = Kp*adjustGain*(error_array[0]-error_array[1]+error_array[0]*delta_time/Ti+Td*(error_array_filter[0]-2*error_array_filter[1]+error_array_filter[2])/delta_time);
  #endif
	control_signal_discrete[0] = control_signal_discrete[1] +control_output_increment;

	if (control_signal_discrete[0]>= max_control) control_signal_discrete[0] = max_control;
	else if (control_signal_discrete[0] <= min_control) control_signal_discrete[0] = min_control;

	error_array[2] = error_array[1];
	error_array[1] = error_array[0];
	control_signal_discrete[1] = control_signal_discrete[0];
	last_time_inSec = current_time;
  #if PIDMODE ==4
	error_array_filter[2]=error_array_filter[1];
	error_array_filter[1]=error_array_filter[0];
  #endif
	last_sensed = sensed_output; // for derivative on PV
	int8_t sign_error    = error_array[0]>0?1:(error_array[0] < 0?-1:0);
  return control_signal_discrete[0];
}
float PID_Discrete3(float *setpoint,float y,float adjustGain, float timer_inSec,float kpValue,float kpDec,float kiValue,float kiDec,float kdValue,float kdDec)
{
	Kp = kpValue/pow(10,kpDec);
	Ki = kiValue/pow(10,kiDec);
	Kd = kdValue/pow(10,kdDec);
  float control_output_increment;
  float current_time = timer_inSec; //returns the number of milliseconds passed since the Arduino started running the program
  #if PIDMODE ==2
  float Ti           = Kp/Ki;
  float delta_time = current_time - last_time_inSec; //delta time interval

	#elif PIDMODE ==3
  float Ti           = Kp/Ki;
  float Td           = Kd/Kp;
  float delta_time = current_time - last_time_inSec; //delta time interval
  #endif

	sensed_output = y;  //measured PV
	float target = *setpoint;
	error_array[0] = target - sensed_output;
//    total_error += error; //accumulates the error - integral term
//    delta_error = error - last_error; //for derivative on error
//    *pdelta_error = -1*(sensed_output - last_sensed); //for derivative on PV

	 /*anti wind-up, the algorithm follows Bisoffi: Reset integral control for improved settling of PID-based motion systems with friction*/



  #if PIDMODE ==1
	control_output_increment = Kp*adjustGain*(error_array[0]-error_array[1]);
  #elif PIDMODE ==2
	control_output_increment = Kp*adjustGain*(error_array[0]-error_array[1]+error_array[0]*delta_time/Ti);
  #elif PIDMODE ==3
	//control_output_increment = Kp*adjustGain*(error_array[0]-error_array[1]+error_array[0]*delta_time/Ti+Td*(error_array[0]-2*error_array[1]+error_array[2])/delta_time);
	control_output_increment = Kp*adjustGain*(error_array[0]-error_array[1]+error_array[0]*delta_time/Ti+Td*(sensed_output-last_sensed-2*error_array[1]+error_array[2])/delta_time);
  //control_output_increment = Kp*adjustGain*(yFilter-last_sensed+error_array[0]*delta_time/Ti+Td*(sensed_output-last_sensed-2*error_array[1]+error_array[2])/delta_time);

  #elif PIDMODE ==4
	error_array_filter[0] = 0.9*error_array_filter[1]+0.1*error_array[1];
	control_output_increment = Kp*adjustGain*(error_array[0]-error_array[1]+error_array[0]*delta_time/Ti+Td*(error_array_filter[0]-2*error_array_filter[1]+error_array_filter[2])/delta_time);
	control_output_increment = Kp*adjustGain*(error_array[0]-error_array[1]+error_array[0]*delta_time/Ti+Td*(error_array_filter[0]-2*error_array_filter[1]+error_array_filter[2])/delta_time);
  #endif
	control_signal_discrete[0] = control_signal_discrete[1] +control_output_increment;

	if (control_signal_discrete[0]>= max_control) control_signal_discrete[0] = max_control;
	else if (control_signal_discrete[0] <= min_control) control_signal_discrete[0] = min_control;

	error_array[2] = error_array[1];
	error_array[1] = error_array[0];
	control_signal_discrete[1] = control_signal_discrete[0];
	last_time_inSec = current_time;
  #if PIDMODE ==4
	error_array_filter[2]=error_array_filter[1];
	error_array_filter[1]=error_array_filter[0];
  #endif
	last_sensed = sensed_output; // for derivative on PV
	int8_t sign_error    = error_array[0]>0?1:(error_array[0] < 0?-1:0);
  return control_signal_discrete[0];
}

/******************************************************************************
* Function Name: FraFreq
* Description : get FRA test frequencies based on linearly or log linear
* Arguments :
* Return Value : derive FRA frequency
******************************************************************************/
void FraFreq(float *freqHz,float freq_start, float freq_stop, float point, uint8_t type)
{
	if (type==1)
	{
	for (int i=0;i<=point;i++)
		{
		freqHz[i] = freq_start+(freq_stop-freq_start)/(point-1) *i;
		 }
	} else if (type==2)
	{
	for (int i=0;i<=point;i++)
		{
		freqHz[i] = pow(10,log10(freq_start)+1/(point-1)*(log10(freq_stop)-log10(freq_start))*i);
		 }
	}
}

/******************************************************************************
* Function Name: olmFraDuty
* Description :
* Arguments : time_each: run time for each frequency
* Return Value : filtered signal
******************************************************************************/
float olmFraFreqSwipeSignal(float amp, float *freqList,float *phaseAcc, float time_each, float *timer_inSec)
{
    int k = floor(*timer_inSec/time_each);
    float phase = 2*3.1415926*freqList[k]*(*timer_inSec);
    *phaseAcc+= phase;
    return amp*sin(phase);
}

/******************************************************************************
* Function Name: discreteFreqSignal_inTime
* Description : sweep freq. under a constant span
* Arguments :
* Return Value : (1)sine wave which each freq. has constant span (2)record the timing of chaning freq.
******************************************************************************/
float discreteFreqSignal_inTime(float amp, float *freqList, float time_each, float timer_inSec, uint8_t *k_stamp )
{
    /* initialize once*/
    static int initialized = 0;
    static float t_reset;
    if(!initialized)
    	{
    	t_reset = timer_inSec;
    	initialized = 1;
    	}

    *k_stamp = floor(timer_inSec/time_each);
    if  (*k_stamp!=k_pre)
    	{
    	t_reset = timer_inSec;
    	}
    float phase = 2*3.1415926*freqList[*k_stamp]*(timer_inSec-t_reset);
    k_pre = *k_stamp;
    return amp*sin(phase);
}
/******************************************************************************
* Function Name: LinearCompensate
* Description : map target to a new one with displacement and adc data of equal span
* Arguments : target to be mapped, supposed unit is in um
* Return Value : mapped target (adc)
******************************************************************************/
double LinearCompensate(double target)
{
	    static double displacement[21]={0,200,400,600,800,1000,1200,1400,1600,1800,2000,2200,2400,2600,2800,3000,3200,3400,3600,3800,4000} ;		
			static double adc[21]={10000,12250,	14500,	16750,	19000,	21250,	23500,	25750,	28000,	30250,	32500,	34750,	37000,	39250,	41500,	43750,	46000,	48250,	50500,	52750,	55000};	
	
			double compensated_setpoint;
			uint8_t flag;
			flag = floor (target/200);
			flag = flag<0?0:(flag> 20?20:flag);	
			compensated_setpoint=adc[flag]+ (target - displacement[flag])/200*(adc[flag+1]-adc[flag]);
			return compensated_setpoint;
}
/******************************************************************************
* Function Name: Binary_Search
* Description : find where the input value is in the array region 
* Arguments : array, input value, left start index of the array (l), right starting index of the array (r), for one 10ea element array, l=0 and r = 9;
* Return Value : mapped target (adc)
******************************************************************************/
int Binary_Search(int array[], int inputValue, int l, int r)
{
    uint8_t mid = l +(r-l)/2;
    if (l>r) return mid;

    if (array[mid]==inputValue)
        return mid;
    else if (array[mid]>inputValue)
        return Binary_Search(array,inputValue,l,mid-1);
    else
        return Binary_Search(array,inputValue,mid+1,r);
}
/******************************************************************************
* Function Name: LinearCompensate2
* Description : map target to a new one with displacement and adc data, span can be inequal
* Arguments : target to be mapped, supposed unit is in um
* Return Value : mapped target (adc)
******************************************************************************/
double LinearCompensate2(double target)
{
//	    static int displacement[23]={0,	162,	324,	455,	583,	749,	881,	1026,	1178,	1318,	1495,	1701,	1849,	2055,	2232,	2412,	2604,	2788,	2971,	3149,	3300,	3506,	3652} ;		
//			static int adc[23]={18060,	18538,	19103,	19598,	20128,	20714,	21224,	21802,	22360,	22883,	23478,	24120,	24568,	25177,	25670,	26194,	26784,	27336,	27868,	28388,	28990,	29587,	30090};	
	
//			static int displacement[12]={0,	324,	583,		881,	1178,	1495,	1849,		2232,	2604,		2971,		3300,		3652} ;		
//			static int adc[12]={28060,		29103,		30128,		31224,		32360,		33478,		34568,		35670,		36784,		37868,		38990,		41090};	
	
//			static int displacement[12]={0,	324,	583,		881,	1178,	1495,	1849,		2232,	2604,		2971,		3300,		3652} ;		
//			static int adc[12]={28060+88000,		29103+88000,		30128+88000,		31224+88000,		32360+88000,		33478+88000,		34568+88000,		35670+88000	,	36784+88000,		37868+88000,		38990+88000,	41090+88000};	

//	
//	
//			static int displacement[6]={0,	583,	1318,		2232,	3149,	3652} ;		
//			static int adc[6]={18060,		20128,	22883,		25670,	28388,30090};		
			
//		  static int displacement[2]={0,4313} ;		
//			static int adc[2]={26831, 41015};	

	static int displacement[2]={0,4000};
	static int adc[2]={10000,55000};
	
//static int displacement[22]={
//0,
////231,
//250,
//428,
//603,
//745,
//898,
//1055,
//1218,
//1385,
//1565,
//1761,
//1963,
//2177,
//2382,
//2590,
//2783,
//3005,
//3185,
//3368,
//3569,
//3771,
//4030
//};
//static int adc[22]={
//3619,
//6361,
//9076,
//11815,
//14560,
//17269,
//20017,
//22734,
//25466,
//28187,
//30920,
//33648,
//36364,
//39099,
//41801,
//44525,
//47266,
//49987,
//52707,
//55452,
//58176,
//60912
//};



//			double compensated_setpoint;
//			uint8_t flag;
//			flag = floor (target/200);
//			flag = flag<0?0:(flag> 20?20:flag);	
//			compensated_setpoint=adc[flag]+ (target - displacement[flag])/200*(adc[flag+1]-adc[flag]);
//			return compensated_setpoint;
				
				double compensated_setpoint;
				uint8_t order;
				order = Binary_Search(displacement,target,0,1);
				compensated_setpoint=adc[order-1]+ (target - displacement[order-1])/(displacement[order]-displacement[order-1])*(adc[order]-adc[order-1]);
				return compensated_setpoint;
}

/******************************************************************************
* Function Name: LinearCompensate3
* Description : map target to a new one with displacement and adc data, disp. and adc data is the argument; the size of disp/adc is used inside function (here is 11)
* Arguments : target to be mapped, disp. and adc point array
* Return Value : mapped target (adc)
******************************************************************************/

double LinearCompensate3(double target, uint8_t *DISP, uint8_t *ADC)
{
				static int adc[11];
				static int displacement[11];
				
	    /* initialize once*/
				static int initialized = 0;
				if(!initialized)
					{
					for (int i=0;i<11;i++){
					adc[i] = (ADC[2*i]<<8) + ADC[2*i+1];
					displacement[i] =(DISP[2*i]<<8) + DISP[2*i+1]; 
					initialized = 1;
					}

				}
	
				double compensated_setpoint;
				uint8_t order;
				order = Binary_Search(displacement,target,0,11-1);
				compensated_setpoint=adc[order-1]+ (target - displacement[order-1])/(displacement[order]-displacement[order-1])*(adc[order]-adc[order-1]);
				return compensated_setpoint;
}
/******************************************************************************
* Function Name: K_Filter
* Description : Kalman Filter for 1D sensor data, x_(k/k-1) = F_(k-1)*x_(k-1)+G_(k-1)*u_(k-1)+w_(k-1); y_k = H_(k-1)*x_k+v_k; 
* Arguments : Reference: A Kalman Filter Tutorial for undergradual studuent
* Return Value : 
******************************************************************************/
double K_Filter(uint32_t yk, int uk, double F, double G, double Q, double R, double x0, double p0)
{
		static double xk, pk;
	  static int initialized = 0;
    if(!initialized)
    	{
			xk = x0;
    	pk = p0;
    	initialized = 1;
    	}
	
		double x_pre;
		double p_pre;
		double K;
		double x_est;
		
		x_pre = F*xk + G*uk;
		p_pre = F*F*pk + Q;
		K     = p_pre/(p_pre+R);
		x_est = x_pre + K*(yk-x_pre);
		
		// update
		pk = (1-K)*p_pre;
		xk = x_est;
		return (xk);
}