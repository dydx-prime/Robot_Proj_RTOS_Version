# Robot_Proj_RTOS_Version
Continuation of the Robot_Proj(make link) repo, where the code is transferred to a TI-RTOS project, with the inclusion of ping pong buffer and line detection.


## Table of Contents

- Overview

- Components Used

- Schematic(s)

- Code Implementation
    - Hardware Interrupts, Software Interrupts, & Semaphores/Tasks
        - Hardware Interrupts
        - Software Interrupts
        - Semaphores/Tasks
        - Priorities & Stack Sizes
    - Global Variables
        - LED
        - command
        - ADC
        - PWM
        - PID
        - Reflect Sensor
        - Ping Pong Buffer
    - main function
    - command functions
    - UART
    - Timers
    - ADC
    - PWM
    - PID
    - Line Detection
    - Ping Pong Buffer

- Results

## Overview

The Robot utilizes a single 9-Volt battery, which powers the motors, sensors, bluetooth module, and microcontroller. The robot is designed to run inside a maze, where it hugs the right wall, and turns right when an intersection is detected. When the robot's front sensor detects a front wall, it will perform a U-turn, and then continue to hug the right wall. The entirely white maze contains black strips of tape, where when the reflect sensor detects the first thin black line, it will assert the ping pong buffer data to output to the bluetooth terminal, until the second thin line is detected. Lastly, when the thick black line is detected, the robot will come to a stop.

## Components Used

The following table contains the relevant parts used, and their description.

| Component | Description |
|-----------|-------------|
| GP2Y0A41SK0F | Analog Left/Front Sensor |
| QTR-1RC | Digital Reflect Sensor |
|EK-TM4C123GXL| Microcontroller    |
|100:1 Gearmotor LP 6V | Motors Driving 40×7mm Wheels |      
|DRV8835 | Dual Motor Driver Carrier      |
|S7V7F5  | 5V Step-Up/Step-Down Regulator |
| HC-05 | Bluetooth Module |

## Schematic(s)

A schematic showcasing how components are connected to each other.

TODO - Create a Map of Where things Go

## Code Implementation

### Hardware Interrupts, Software Interrupts, & Semaphores/Tasks

#### Hardware Interrupts

// FINISH

#### Software Interrupts
The following table contains the name of the software interrupts and description.

| Software Interrupt | Description |
|--------------------|-------------|
| stopSWI            | Calls a function to stop the wheels, which then blinks the red LED for 1 minute, and then exists the program|


#### Semaphores/Tasks

The following table contains the name of the semaphores, tasks, and description.

|         Semaphores      | Tasks | Description |
|-------------------------|-------|-------------|
| command_semaphore       |       | Executes the function associated with the user typed command.
| front_semaphore         |       | Reads front sensor data, and executes the U-turn function when criteria is met
| pid_semaphore           |       | Reads right sensor data and executes the right turn function if criteria is met. Otherwise, it calls the PID function.
| reflect_sensor_semaphore|       | Reads the value of the reflect sensor, controls ping pong buffer data ouput, and stops the robot upon detecting a thick black line.
| storePIDError_semaphore |       | Stores PID error in a ping pong buffer, which is sent once the buffer is full and active.

#### Priorities & Stack Sizes

// FINISH

### Global Variables

#### LED

```c
// LED Switching on Terminal Inputs
/// PF1 = RED, PF2 = BLUE, PF3 = RED
uint8_t pins[] = {GPIO_PIN_3, GPIO_PIN_2, GPIO_PIN_1} 
int led_count = 1;
```

#### command

```c
volatile bool command_recieved = false;
volatile int command_char_count = 0;
char input_cmd[4];
char prev_input_cmd[4] = "NA_";
```

#### ADC

```c
volatile uint32_t ADCAvVal;
volatile uint32_t ADCAvVal2;
volatile bool uTurnStatus = false;
volatile float volts1, right_distance; // right sensor
volatile float volts2, front_distance; // front sensor
```

#### PWM

```c
uint32_t ui32PWMClock;
uint32_t ui32Load;
volatile int baseDuty = 98; // duty cycle speed %
```

#### PID

```c
volatile float target_distance = 10;
volatile float curr_error = 0.0, prev_error = 0.0, integral_error = 0.0;
volatile float Kp = 1.2, Ki = 0.50, Kd = 0.25;
volatile float proportional = 0.0, integral = 0.0, derivative = 0.0, adjustment = 0.0;
```

#### Reflect Sensor

```c
volatile uint8_t NumOfLinesCount = 0;
const uint32_t single_black_threshold = 100000U;
const uint32_t thick_black_threshold = 280000U;
uint32_t startVal = 0;
uint32_t endVal = 0;
volatile bool prevBlackFlag = false;
volatile bool blackFlag = false;
```

#### Ping Pong Buffer

```c
uint8_t PingPongBuf[2][20];
uint8_t txBuf = 1;
uint8_t activateBuf = 0;
uint8_t sampleIndex = 0;
volatile bool pid_error_flag = false;
volatile bool pingPong = false;
```

### Main Function

The main function is responsible for enabling peripherals such as GPIO, Timers, ADC, PWM, and more (directly through the function call, or indirectly through other function calls). The initial state of the program is to hold a static green LED on the microcontroller, as it serves as  an indicator that the first input for a command is ready.

```c
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
```

### UART

```c

```

### Command Functions

A typedef structure is instantiated to create the 3-letter command (in addition to the null termintor character) and a pointer to the related function. Thus, a lookup table is needed to map the command and the associated function. A single entry is placed in this case, so the table_entries variable is redundant, but necessary when adding more entries. Below the lookup table is the **start()** function, turns off the LEDs and enables the 50ms timer, Timer1A. To execute the command, the task **execute_cmd_task** calls the function **execute_command()**, which takes a string that will be compared to the previous command used to avoid calling the same function twice. An output message will notify the user that the command cannot be used again (in succession). Once that case passes, the string will be checked to match one of the commands in the lookup table, and executes it. If a match is not found, then an output message is sent to notify the user that the command does not exist. Each output message is sent to UART1, the bluetooth terminal. The **execute_cmd_task** then resets the command character count as well as the recieved flag to allow the next command to be read.

```c
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

```

### Timers

### ADC

### PWM

### PID

### Ping-Pong Buffer

### Line Detection

### Ping-Pong Buffer


## Results






