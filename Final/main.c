
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

extern ti_sysbios_knl_Semaphore_Handle command_semaphore;
extern ti_sysbios_knl_Semaphore_Handle front_semaphore;
extern ti_sysbios_knl_Semaphore_Handle pid_semaphore;


//------ Globals ---------
// LED Switching on Terminal Inputs
uint8_t pins[] = {GPIO_PIN_3, GPIO_PIN_2, GPIO_PIN_1}; // FP1 = RED, PF2 = BLUE, PF3 = GREEN
int led_count = 1;

// command variables
volatile bool command_received = false;
volatile int command_char_count = 0;
char input_cmd[4];
char prev_input_cmd[4] = "NA_";

// adc
volatile uint32_t ADCAvVal;
volatile uint32_t ADCAvVal2;
//bool front_block_flag = false;
bool uTurnStatus = false;
volatile float volts1, right_distance; // right sensor
volatile float volts2, front_distance; // front sensor

// pwm
uint32_t ui32PWMClock;
uint32_t ui32Load;
volatile int baseDuty = 50;   // duty cycle speed %

// pid
volatile float target_distance = 8.5;  // target distance in cm
volatile float curr_error = 0.0, prev_error = 0.0, integral_error = 0.0;
//volatile float Kp = 0.8, Ki = 0.005, Kd = 0.08;
volatile float Kp = 1.2, Ki = 0.05, Kd = 0.05;
volatile float proportional = 0.0, integral = 0.0, derivative = 0.0, adjustment = 0.0;

// -- function declaration
void UART_init();
void ADC_init();
void PWM_init();
void Timer1A_init();
void Timer1AIntHandler(void);
void UART0IntHandler(void);
void UART1IntHandler(void);
void execute_command(char *cmd);
void start();
void ADCSeq3IntHandler(void);
void ADCSeq2IntHandler(void);
void execute_pid_task(UArg a0, UArg a1);
void execute_front_task(UArg a0, UArg a1);
void execute_cmd_task(UArg a0, UArg a1);
void pid_func(void);
void uTurn(void);
void rightTurn(void);
float right_calc(void);


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
//	if(strcmp(cmd, prev_input_cmd) == 0){
//		char terminal_string[] = "\n\rCommand cannot be used again!\r\n";
//		return;
//	}

	int i; // like, whyy?
	for(i = 0; i < table_entries; i++){
        if (strcmp(cmd, lookup_table[i].cmd) == 0) {
            lookup_table[i].cmd_funct(); // run the matched function

            strcpy(prev_input_cmd, cmd);
            return;
        }
	}
//	char terminal_string[] = "\n\rCommand does not exist!\r\n";
//	UARTSend((uint8_t *) terminal_string, strlen(terminal_string));

}


// -- functions implemented
void ADCSeq3IntHandler(void)
{
    ADCIntClear(ADC0_BASE, 3);

    /* Post PID semaphore for task-level processing */
    Semaphore_post(pid_semaphore);
}

void ADCSeq2IntHandler(void)
{
	ADCIntClear(ADC0_BASE, 2);

    /* Post front sensor semaphore */
	Semaphore_post(front_semaphore);
}

void Timer1AIntHandler(void){
	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	ADCProcessorTrigger(ADC0_BASE, 2);
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

        /* LED cycling (same as before) */
        GPIOPinWrite(GPIO_PORTF_BASE, pins[led_count], pins[led_count]);
        GPIOPinWrite(GPIO_PORTF_BASE, pins[(led_count+1)%3], 0);
        GPIOPinWrite(GPIO_PORTF_BASE, pins[(led_count+2)%3], 0);
        led_count = (led_count + 1) % 3;

        /* fill command buffer */
        input_cmd[command_char_count] = c;
        command_char_count++;

        if (command_char_count == 3) {
            input_cmd[3] = '\0';
            command_received = true;
            Semaphore_post(command_semaphore);
        }
    }
}

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

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));

	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

	//-----Sensor1----- // Right Sensor
	// PE2 as an analog input pin
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);
	ADCSequenceDisable(ADC0_BASE, 3);

	// ADC0 sequence 3 triggered by processor
	ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH1);
	ADCSequenceEnable(ADC0_BASE, 3);
	// register interrupt handler for ADC0 sequence 3
	//ADCIntRegister(ADC0_BASE, 3, &ADCSeq3IntHandler);
	ADCIntEnable(ADC0_BASE, 3);


	//-----Sensor2----- // Front Sensor
	// PE1 as an analog input pin
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1);
	ADCSequenceDisable(ADC0_BASE, 2);

	// ADC0 sequence 2 triggered by processor
	ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH2);
	ADCSequenceEnable(ADC0_BASE, 2);
	// register interrupt handler for ADC0 sequence 2

	//ADCIntRegister(ADC0_BASE, 2, &ADCSeq2IntHandler);
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
	int TIMER_PERIOD_MS = 50;
	uint32_t ui32Period = (SysCtlClockGet() / 1000) * TIMER_PERIOD_MS;
	TimerLoadSet(TIMER1_BASE, TIMER_A, ui32Period - 1);

	TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	IntEnable(INT_TIMER1A);

}

/*
 *  ======== main ========
 */
int main(void){

    FPUEnable();
    FPULazyStackingEnable();

	// 50 Mhz main clock
	SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

	IntMasterEnable();

	// enable GREEN LED
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_3);

	// UART Enable
	UART_init();

    // ADC Enable
	ADC_init();

    // PWM Enable
    PWM_init();

    Timer1A_init();

    /* Start BIOS */
    BIOS_start();

    return (0);
}

// implemented tasks
void execute_cmd_task(UArg a0, UArg a1)
{
    (void)a0; (void)a1;

    while (1) {
        Semaphore_pend(command_semaphore, BIOS_WAIT_FOREVER);

        /* execute the command (input_cmd is filled by UART1 ISR) */
        execute_command((char *)input_cmd);

        /* reset flags counters (same logic as original main loop) */
        command_char_count = 0;
        command_received = false;
    }
}

void execute_pid_task(UArg a0, UArg a1)
{
    (void)a0; (void)a1;

    while (1) {
        Semaphore_pend(pid_semaphore, BIOS_WAIT_FOREVER);

        /* Read ADC sequence 3 value (right sensor) */
        ADCSequenceDataGet(ADC0_BASE, 3, &ADCAvVal);
        volts1 = ADCAvVal * (3.3 / 4096.0);
        right_distance = 5.0685 * pow(volts1, 2) - 23.329 * volts1 + 31.152;

        /* Use the same logic you had originally */
        if (right_distance > target_distance) {
            rightTurn();
        } else {
            pid_func();
        }

        /* Re-enable ADC interrupt if you rely on it elsewhere (same as before) */
        ADCIntEnable(ADC0_BASE, 3);
    }
}

void execute_front_task(UArg a0, UArg a1)
{
    (void)a0; (void)a1;

    while (1) {
        Semaphore_pend(front_semaphore, BIOS_WAIT_FOREVER);

        /* Read ADC sequence 2 value (front sensor) */
        ADCSequenceDataGet(ADC0_BASE, 2, &ADCAvVal2);
        volts2 = ADCAvVal2 * (3.3/4096.0);
        front_distance = 5.0685 * pow(volts2, 2) - 23.329 * volts2 + 31.152;

        if (!uTurnStatus && front_distance <= 9) {
            uTurnStatus = true;
            uTurn();
        }
        else if (uTurnStatus && (front_distance > 15) ) {
            uTurnStatus = false;

            /* set wheels forward again */
            GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5 , 0);
            GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4 , 0);

            int rightDuty = baseDuty;
            int leftDuty  = baseDuty;

            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (ui32Load * rightDuty) / 100);
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, (ui32Load * leftDuty) / 100);
        }
        else if(!uTurnStatus) {
            // chain time
            ADCProcessorTrigger(ADC0_BASE, 3);
        }

        ADCIntEnable(ADC0_BASE, 2);
    }
}


void pid_func(void){

	// enable green LED
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_3);

    // --- PID ---
    prev_error = curr_error;
    curr_error = target_distance - right_distance;

    // clamp integral error
    integral_error += curr_error * .05;
    if(integral_error > 100) integral_error = 100;
    if(integral_error < -100) integral_error = -100;

    proportional = Kp * curr_error;
    derivative = Kd * (curr_error - prev_error) / .05; // d/dt
    integral = Ki * integral_error;

    adjustment = proportional + integral + derivative;

    // --- Motor Duty Cycles ---
    int rightDuty = baseDuty + adjustment;
    int leftDuty  = baseDuty - adjustment;

    // clamps
    if (rightDuty > 80) rightDuty = 80;
    if (rightDuty < 20) rightDuty = 20;

    if (leftDuty  > 80) leftDuty = 80;
    if (leftDuty  < 20) leftDuty = 20;

    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (ui32Load * rightDuty) / 100);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, (ui32Load * leftDuty) / 100);

}

void rightTurn(void){

	// enable BLUE LED
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_2);

    //40 / 60
    int rightDuty = 40;
    int leftDuty  = 80;

    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (ui32Load * rightDuty) / 100);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, (ui32Load * leftDuty) / 100);

}

void uTurn(void){

	// enable RED LED
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_1);

	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5 , 0); // right motor phase - forward
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4 , GPIO_PIN_4); // left motor phase - backwards

    // experiment with duty cycles - try to get the robot to spin in place
    int rightDuty = 75;
    int leftDuty  = 75;

    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (ui32Load * rightDuty) / 100);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, (ui32Load * leftDuty) / 100);

}
