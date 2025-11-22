/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Swi.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>


#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/pwm.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"

//------------ semapahore/swi declaration ---------------//

extern ti_sysbios_knl_Semaphore_Handle command_semaphore;
extern ti_sysbios_knl_Semaphore_Handle front_semaphore;
extern ti_sysbios_knl_Semaphore_Handle pid_semaphore;
extern ti_sysbios_knl_Semaphore_Handle pid_error_semaphore;
extern ti_sysbios_knl_Semaphore_Handle reflect_sensor_semaphore;
extern ti_sysbios_knl_Semaphore_Handle storePIDError_semaphore;
extern const Swi_Handle stopSWI;



//------------ Globals ---------------//

// LED Switching on Terminal Inputs
uint8_t pins[] = {GPIO_PIN_3, GPIO_PIN_2, GPIO_PIN_1}; // FP1 = RED, PF2 = BLUE, PF3 = GREEN
int led_count = 1;

// command stuff
volatile bool command_received = false;
volatile int command_char_count = 0;
char input_cmd[4];
char prev_input_cmd[4] = "NA_";

// adc
volatile uint32_t ADCAvVal;
volatile uint32_t ADCAvVal2;
volatile bool uTurnStatus = false;
volatile float volts1, right_distance; // right sensor
volatile float volts2, front_distance; // front sensor

// pwm
uint32_t ui32PWMClock;
uint32_t ui32Load;
volatile int baseDuty = 98;   // duty cycle speed %

// pid
volatile float target_distance = 10;  // target distance in cm
volatile float curr_error = 0.0, prev_error = 0.0, integral_error = 0.0;
//volatile float Kp = 1.2, Ki = 0.05, Kd = 0.05;
volatile float Kp = 1.2, Ki = 0.50, Kd = 0.25;
volatile float proportional = 0.0, integral = 0.0, derivative = 0.0, adjustment = 0.0;

// reflect sensooooor
volatile uint8_t NumOfLinesCount = 0;
const uint32_t single_black_threshold = 100000U;
const uint32_t thick_black_threshold = 280000U;
uint32_t startVal = 0;
uint32_t endVal = 0;
volatile bool prevBlackFlag = false;
volatile bool blackFlag = false;

// ping-pong bufffffffffffffffffffffffffffer
uint8_t PingPongBuf[2][20];
uint8_t txBuf = 1;
uint8_t activateBuf = 0;
uint8_t sampleIndex = 0;
volatile bool pid_error_flag = false;
volatile bool pingPong = false;




//------------ function declaration (in order from top to bottom) ---------------//
// ping pong
void AssembleFrame(uint8_t *buffer);
void StorePIDError(float);
void TransmitTask(UArg arg0, UArg arg1);
void SendFrameUART(char *Frame);

//command
void start(void);
void execute_command(char *cmd);
void execute_cmd_task(UArg a0, UArg a1);

// interrupt handlers
void ADCSeq3IntHandler(void);
void ADCSeq2IntHandler(void);
void Timer1AIntHandler(void);
void UART0IntHandler(void);
void UART1IntHandler(void);

// init
void UART_init();
void ADC_init();
void PWM_init();
void Timer1A_init();
void ReflectSensorTimerInit(void);

//reflect sensor
uint32_t ReflectSensorValue(void);
void execute_sensor_task(UArg a0, UArg a1);
void stop(UArg a0, UArg a1);
void blinky(int counter);

// adc/timer/pid/everything that makes the robot run
void execute_pid_task(UArg a0, UArg a1);
void execute_front_task(UArg a0, UArg a1);
void pid_func(void);
void rightTurn(void);
void uTurn(void);
void UART1Send(const uint8_t *pui8Buffer, uint32_t ui32Count);



//------------ pingy pong functions / tasks ---------------//

void AssembleFrame(uint8_t *buffer){
	char Frame[100];
	int offset = 0;
	strcpy(Frame, "Team11: ");
	offset = strlen(Frame);

	int i;
	for (i = 0; i < 20; i++){
		uint8_t value = buffer[i];
		uint8_t tens = value / 10;
		uint8_t ones = value % 10;
		Frame[offset++] = tens + '0';
		Frame[offset++] = ones + '0';
		Frame[offset++] = ' ';
	}
	Frame[offset++] = '\r';
	Frame[offset++] = '\n';
	Frame[offset++] = '\0';

	SendFrameUART(Frame);
}


void StorePIDErrorTask(UArg arg0, UArg arg1){

	while (1){
		Semaphore_pend(storePIDError_semaphore, BIOS_WAIT_FOREVER);

		float mm = curr_error * 10.0f;
			uint8_t data = (uint8_t) fabs(mm);

			PingPongBuf[activateBuf][sampleIndex++] = data;

			if (sampleIndex >= 20){
				sampleIndex = 0;
				Semaphore_post(pid_error_semaphore);

				uint8_t temp = activateBuf;
				activateBuf = txBuf;
				txBuf = temp;
			}
	}
}

void TransmitTask(UArg arg0, UArg arg1){

    while (1) {
        Semaphore_pend(pid_error_semaphore, BIOS_WAIT_FOREVER);
        AssembleFrame(PingPongBuf[txBuf]);
    }

}


void SendFrameUART(char *Frame){
	int i = 0;
	while (Frame[i] != '\0'){
		UARTCharPut(UART1_BASE, Frame[i]);
		i++;
	}
	UARTCharPut(UART1_BASE, '\n');
}



//------------ command functions / tasks ---------------//

// typedef struct
typedef void (*Fn)(void);
typedef struct {
	char cmd[4]; // command + '/0'
	Fn cmd_funct; // pointer to command function
}lookup;

// lookup table
lookup lookup_table[] = {
	{"STR", start}
};

int table_entries = sizeof(lookup_table) / sizeof(lookup_table[0]);

void start(){
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);
    TimerEnable(TIMER1_BASE, TIMER_A);
}

void execute_command(char *cmd){
	if(strcmp(cmd, prev_input_cmd) == 0){
		char terminal_string[] = "\n\rCommand cannot be used again!\r\n";
		return;
	}

	int i; // like, whyy?
	for(i = 0; i < table_entries; i++){
        if (strcmp(cmd, lookup_table[i].cmd) == 0) {
            lookup_table[i].cmd_funct(); // run the matched function

            strcpy(prev_input_cmd, cmd);
            return;
        }
	}
	char terminal_string[] = "\n\rCommand does not exist!\r\n";
	UART1Send((uint8_t *) terminal_string, strlen(terminal_string));

}

void execute_cmd_task(UArg a0, UArg a1)
{
    (void)a0; (void)a1;

    while (1) {
        Semaphore_pend(command_semaphore, BIOS_WAIT_FOREVER);

        execute_command((char *)input_cmd);

        /* reset flags counters */
        command_char_count = 0;
        command_received = false;
    }
}



//------------ interrupt handler functions (adc, timers) ---------------//

void ADCSeq3IntHandler(void){
    ADCIntClear(ADC0_BASE, 3);
    Semaphore_post(pid_semaphore);
}

void ADCSeq2IntHandler(void){
	ADCIntClear(ADC0_BASE, 2);
	Semaphore_post(front_semaphore);
}

// runs every 50 ms
void Timer1AIntHandler(void){
	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	ADCProcessorTrigger(ADC0_BASE, 2);

	//line detection
	Semaphore_post(reflect_sensor_semaphore);

	// pid error flag ensures it gets called every 100 ms
	// pingPong flag updated in the reflect sensor task
	if(pid_error_flag && pingPong){
		Semaphore_post(storePIDError_semaphore);
		//StorePIDError(curr_error);
	}
	pid_error_flag = !pid_error_flag;
}

void UART0IntHandler(void){
    uint32_t ui32Status;

    ui32Status = UARTIntStatus(UART0_BASE, true);
    UARTIntClear(UART0_BASE, ui32Status);

    while (UARTCharsAvail(UART0_BASE)) {
        char c = UARTCharGetNonBlocking(UART0_BASE);
        UARTCharPutNonBlocking(UART1_BASE, c);
        UARTCharPutNonBlocking(UART0_BASE, c);
    }
}

void UART1IntHandler(void){
    uint32_t ui32Status;

    ui32Status = UARTIntStatus(UART1_BASE, true);
    UARTIntClear(UART1_BASE, ui32Status);

    while (UARTCharsAvail(UART1_BASE)) {
        char c = UARTCharGetNonBlocking(UART1_BASE);

        UARTCharPutNonBlocking(UART0_BASE, c);
        UARTCharPutNonBlocking(UART1_BASE, c);

        GPIOPinWrite(GPIO_PORTF_BASE, pins[led_count], pins[led_count]);
        GPIOPinWrite(GPIO_PORTF_BASE, pins[(led_count+1)%3], 0);
        GPIOPinWrite(GPIO_PORTF_BASE, pins[(led_count+2)%3], 0);
        led_count = (led_count + 1) % 3;

        // command buffer
        input_cmd[command_char_count] = c;
        command_char_count++;

        if (command_char_count == 3) {
            input_cmd[3] = '\0';
            command_received = true;
            Semaphore_post(command_semaphore);
        }
    }
}


//------------ init functions (uart, adc, pwm, timers) ---------------//

void UART_init(){
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	// UART0 pins A0, A1 ; RX, TX
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// UART1 pins B0, B1 ; RX, TX
	GPIOPinConfigure(GPIO_PB0_U1RX);
	GPIOPinConfigure(GPIO_PB1_U1TX);
	GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
	                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
	                             UART_CONFIG_PAR_NONE));

	UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 9600,
	                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
	                             UART_CONFIG_PAR_NONE));

	IntEnable(INT_UART0);
	UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);

	IntEnable(INT_UART1);
	UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);
}

void ADC_init(){

	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));

	//-----Sensor1----- // Right Sensor
	// PE2 as an analog input pin
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);
	ADCSequenceDisable(ADC0_BASE, 3);

	// ADC0 sequence 3 triggered by processor
	ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH1);
	ADCSequenceEnable(ADC0_BASE, 3);
	ADCIntEnable(ADC0_BASE, 3);


	//-----Sensor2----- // Front Sensor
	// PE1 as an analog input pin
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1);
	ADCSequenceDisable(ADC0_BASE, 2);

	// ADC0 sequence 2 triggered by processor
	ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH2);
	ADCSequenceEnable(ADC0_BASE, 2);
	ADCIntEnable(ADC0_BASE, 2);

}

void PWM_init(){

	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0));
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));

	SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

	// phase pins - LOW = forward, HIGH = reverse
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_5 ); // right motor phase
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_4 ); // left motor phase

	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5 , 0);
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4 , 0);
	// Note that for the phase pins above, if you set both to high, a bug is created (idk y)

	// PWM pins for motors
	GPIOPinConfigure(GPIO_PB6_M0PWM0); // right wheel
	GPIOPinConfigure(GPIO_PB4_M0PWM2); // left wheel

	GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);
	GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4);


	ui32PWMClock = SysCtlClockGet() / 64;
	int PWM_FREQUENCY = 5000;
	ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;

	// right wheel
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32Load);
	PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (ui32Load * 1) / 100); // duty cycle is 1
	PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
	PWMGenEnable(PWM0_BASE, PWM_GEN_0);

	// left wheel
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, ui32Load);
	PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, (ui32Load * 1) / 100); // duty cycle is 1
	PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
	PWMGenEnable(PWM0_BASE, PWM_GEN_1);

}

void Timer1A_init(){
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER1));

	TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
	int TIMER_PERIOD_MS = 50; // 50 ms
	uint32_t ui32Period = (SysCtlClockGet() / 1000) * TIMER_PERIOD_MS;
	TimerLoadSet(TIMER1_BASE, TIMER_A, ui32Period - 1);

	TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	IntEnable(INT_TIMER1A);
}

void ReflectSensorTimerInit(void){
    // enable WTIMER1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER1);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_WTIMER1));

    TimerIntDisable(WTIMER1_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntClear(WTIMER1_BASE, TIMER_TIMA_TIMEOUT);

    // configure as down counter
    TimerDisable(WTIMER1_BASE, TIMER_A);
    TimerConfigure(WTIMER1_BASE, TIMER_CFG_A_PERIODIC);
    TimerLoadSet(WTIMER1_BASE, TIMER_A, 0xFFFFFFFF);
    TimerEnable(WTIMER1_BASE, TIMER_A);
}

//------------ reflect sensor functions/tasks ---------------//

void execute_sensor_task(UArg a0, UArg a1){

    while (1){

        Semaphore_pend(reflect_sensor_semaphore, BIOS_WAIT_FOREVER);

        // ---- sensor value ----
        // charge sensor - PB7
        GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_7);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, GPIO_PIN_7);

        // charge for 10 microseconds
        SysCtlDelay(SysCtlClockGet() / 300000);

        // set pin as input to discharge
        GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_7);
        startVal = TimerValueGet(WTIMER1_BASE, TIMER_A);

        // delay for pin to reach low
        while (GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_7) != 0) {
            endVal = TimerValueGet(WTIMER1_BASE, TIMER_A);
            if ( (startVal - endVal) > 300000U) {break;}
        }
        endVal = TimerValueGet(WTIMER1_BASE, TIMER_A);

        uint32_t sensor_val = (startVal - endVal);

        // ---- line detection ----
        if(sensor_val > single_black_threshold){
        	blackFlag = true;
        }
        else{
        	blackFlag = false;
        }
        // black line detection for a previous white line value
        if (blackFlag && !prevBlackFlag){

        	NumOfLinesCount++;

            if (NumOfLinesCount == 1){
            	// GREEN LED, ping pong enable
            	pingPong = true;
            	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_3);
            }
            else if (NumOfLinesCount == 2){
            	// BLUE LED, ping pong disable
            	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_2);

                // send partial ping pong data
            	Semaphore_post(pid_error_semaphore);
                pingPong = false;
            }
        }
        // detection of thick line
        if (NumOfLinesCount >= 2 && (sensor_val > thick_black_threshold) ){
        	// call swi to stop bot
			Swi_post(stopSWI);
		}
        prevBlackFlag = blackFlag;
    }
}



void stopRobot(UArg arg0, UArg arg1) {

		// disable interrupts
		IntMasterDisable();

		// RED LED
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_1);

		// stop wheels
		int rightDuty = 2;
		int leftDuty  = 2;
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (ui32Load * rightDuty) / 100);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, (ui32Load * leftDuty) / 100);

		// blink RED LED for ~ 1 minute
		blinky(300); // can adjust value to increase or reduce time blinking

		// LED off
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);

		// exit program
		BIOS_exit(0);

 }

void blinky(int counter){

	while (counter > 0){

		int i;
		// Delay for a bit.
	    for(i = 0; i < 200000; i++){}

	    // LED off.
	    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);

	    // Delay for a bit.
	    for(i = 0; i < 200000; i++){}

	    // set to RED LED
	    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
	    counter--;

	}

}

//------------ adc/timer function/tasks ---------------//

// right sensor
void execute_pid_task(UArg a0, UArg a1){
    (void)a0; (void)a1;

    while (1) {
        Semaphore_pend(pid_semaphore, BIOS_WAIT_FOREVER);

        // Read ADC sequence 3 val & convert to cm
        ADCSequenceDataGet(ADC0_BASE, 3, &ADCAvVal);
        volts1 = ADCAvVal * (3.3 / 4096.0);

        // faster than pow()
        float volts1_squared = volts1 * volts1;
        right_distance = 5.0685 * volts1_squared - 23.329 * volts1 + 31.152;

        if (right_distance > target_distance) {
            rightTurn();
        }

        else {
            pid_func();
        }

    }
}

void execute_front_task(UArg a0, UArg a1)
{
    (void)a0; (void)a1;

    while (1) {
        Semaphore_pend(front_semaphore, BIOS_WAIT_FOREVER);

        // Read ADC sequence 2 val & convert to cm
        ADCSequenceDataGet(ADC0_BASE, 2, &ADCAvVal2);
        volts2 = ADCAvVal2 * (3.3/4096.0);

        // faster than pow()
        float volts2_squared = volts2 * volts2;
        front_distance = 5.0685 * volts2_squared - 23.329 * volts2 + 31.152;

        if (!uTurnStatus && front_distance <= 4.5f) {
            uTurnStatus = true;
            uTurn();
        }

        else if (uTurnStatus && (front_distance > 15.0f) ) {
            uTurnStatus = false;

            // set wheels forward again
            GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5 , 0);
            GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4 , 0);

            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (ui32Load * baseDuty) / 100);
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, (ui32Load * baseDuty) / 100);
        }

        else if(!uTurnStatus) {
        	// calls ADCSeq3IntHandler
            ADCProcessorTrigger(ADC0_BASE, 3);
        }

    }
}


void pid_func(void){

	// enable green LED
	// GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);
	// GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_3);

    // --- PID ---
    prev_error = curr_error;
    curr_error = target_distance - right_distance;

    // clamp integral error
    integral_error += curr_error * .05;
    if(integral_error > 100) integral_error = 100;
    if(integral_error < -100) integral_error = -100;

    proportional = Kp * curr_error;
    derivative = Kd * (curr_error - prev_error) / .05;
    integral = Ki * integral_error;

    adjustment = proportional + integral + derivative;

    // --- Motor Duty Cycles ---
    int rightDuty = baseDuty + adjustment;
    int leftDuty  = baseDuty - adjustment;

    // clamps // 80
    if (rightDuty > 80) rightDuty = 98;
    if (rightDuty < 20) rightDuty = 20;

    if (leftDuty  > 80) leftDuty = 98;
    if (leftDuty  < 20) leftDuty = 20;

    // update wheels
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (ui32Load * rightDuty) / 100);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, (ui32Load * leftDuty) / 100);

}

void rightTurn(void){

//  enable BLUE LED
//  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);
//  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_2);

    int rightDuty = 30;
    int leftDuty  = 80;

    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (ui32Load * rightDuty) / 100);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, (ui32Load * leftDuty) / 100);

}

void uTurn(void){

//  enable RED LED
//	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);
//  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_1);

	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5 , 0); // right motor phase - forward
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4 , GPIO_PIN_4); // left motor phase - backwards

	// 75
    int rightDuty = 90;
    int leftDuty  = 90;

    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (ui32Load * rightDuty) / 100);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, (ui32Load * leftDuty) / 100);

}


//------------ miscellaneous ---------------//
void UART1Send(const uint8_t *pui8Buffer, uint32_t ui32Count){
    // Loop while there are more characters to send.
    while(ui32Count != 0)
    {
    	ui32Count--;
        UARTCharPut(UART1_BASE, *pui8Buffer++);
    }
}



//------------ MAIN ---------------//
  int main(void){

    FPUEnable();
    FPULazyStackingEnable();

	// clock
	SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

	IntMasterEnable();

	// enable GREEN LED
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_3);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));

	// UART Enable
	UART_init();

    // ADC Enable
	ADC_init();

    // PWM Enable
    PWM_init();

    // Reflect Sensor Timer Enable
    ReflectSensorTimerInit();

    // Timer For Everything Else Enable
    Timer1A_init();

    // Start BIOS
    BIOS_start();

    return (0);
}
