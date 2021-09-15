// Infrared remote control and module (NEC protocol)

// We are using the Tiva C, Breadboard (B), infrared module, and 5V from the power supply.
// Tiva C Connections: GND to B(GND), PC4 to infrared Y pin, USB to PC
// Power Supply Connections: +5V to infrared R pin, GND to infrared G pin.

// Make sure the power supply is on; otherwise you will not get a reading.

// Tiva C components used: UART0, PC4 (for reading from the infrared module), WTIMER0A (pulse measurement), SysTick (delays) 

// In system_TM4C123.c, CLOCK_SETUP = 0; we are using 16MHz clock.

#include "stdio.h"
#include "stdlib.h"

#include "TM4C123GH6PM.h"
#include "gpiortns.h"
#include "uartrtns.h"
#include "driverlib/sysctl.h"

// Provide for receiving 64 bits of data
#define MAX_DATA 64
static volatile uint32_t TimerValueArr[64];
static volatile uint32_t TimerValueIdx;

static volatile int32_t CurrentTicks;

typedef struct
{
    uint32_t value;
    char button[7 + 1]; // Made 7 + 1 for structure alignment
} ir_data_t;

static ir_data_t NECData[] =
{ 
    { 41565, "PWR" },
    { 25245, "VOL+" },
    { 57885, "FN/STOP" },
    { 8925,  "REWD" },
    { 765,   "PLAY" },
    { 49725, "FFWD" },
    { 57375, "DARR" },
    { 43095, "VOL-" },
    { 36975, "UARR" },
    { 26775, "0" },
    { 39015, "EQ" },
    { 45135, "ST/REPT" },
    { 12495, "1" },
    { 6375,  "2" },
    { 31365, "3" },
    { 4335,  "4" },
    { 14535, "5" },
    { 23205, "6" },
    { 17085, "7" },
    { 19125, "8" },
    { 21165, "9" },
};

// Internal function prototypes
static void setup_uart0(void);
static void printChar(const char c);
static void printString(const char * string);

static void sensor_input(void);
static void show_result(void);
static uint16_t show_nec_result(const uint32_t timer_data_arr[]);
static char * get_nec_reading_1(const uint32_t binary_data_arr[]);
static char * get_nec_reading_2(const uint32_t binary_data_arr[]);
static uint32_t b2i(const uint32_t binary_data_arr[], const uint32_t r);
static uint32_t mypow(const uint32_t i);

static void setup_systick(void);
static void initTimerValueArr(void);

void SysTick_Handler(void);
void WTIMER0A_Handler(void);

int main(void)
{
    setup_uart0();
//  To call SysCtlClockGet():
//  - Change driverlib/sysctl.h to include <inc/hw_types.h>
//  - Change the calling program to include "driverlib/sysctl.h"
//  - Add driverlib-cm4f.lib to the project (to prevent linker error due to missing SysCtlClockGet())

    // Show clock speed
    char clockString[40 + 1];
    sprintf(clockString, "\n\r%lu\n\r", SysCtlClockGet());
    printString(clockString);

    sensor_input();
    
    // This must be the last step (unless some prior code requires CurrentTicks). 
    setup_systick();
    int32_t previousClockTicks = 0;

    initTimerValueArr();    

    while (1)
    {
        int32_t now = CurrentTicks;
        
        if (TimerValueArr[0] > 0 && previousClockTicks == 0)
        {
            previousClockTicks = now;
        }
        else if (previousClockTicks && now - previousClockTicks > 200) // Wait 200ms to receive all data
        {
            show_result();
            initTimerValueArr();
            previousClockTicks = 0;
            WTIMER0->TAV = 0;
        }            
    }

    // Commented to prevent compiler warning
    // return(0);
}

// Initialise TimerValueArr.
static void initTimerValueArr(void)
{
    TimerValueIdx = 0;
    for (uint32_t i = 0; i < MAX_DATA; i++)
        TimerValueArr[i] = 0;
}

// Show command.
static void show_result(void)
{
    // Using TimerValueArr, we will first compute the duration into binary_data_arr. In a further step,
    // we will convert the duration into either a 0 or 1.
    uint32_t timer_data_arr[64];
    for (uint32_t i = 0; i < MAX_DATA; i++)
        timer_data_arr[i] = 0;

    for (uint32_t i = 0; i < MAX_DATA && i + 1 < MAX_DATA; i++)
    {
        if (TimerValueArr[i + 1] == 0)
            break;
        timer_data_arr[i] = TimerValueArr[i + 1] - TimerValueArr[i];

        // Debug code to compare low duration values with high ones and use it the next step.
//        char mesg[40 + 1];
//        sprintf(mesg, "%d: %d\n\r", i, timer_data_arr[i]);
//        printString(mesg);
    }
    
    uint16_t reading = show_nec_result(timer_data_arr);
    if (reading) return;
}

// Process NEC encoding format.
static uint16_t show_nec_result(const uint32_t timer_data_arr[])
{
    uint32_t binary_data_arr[64];
    for (uint32_t i = 0; i < MAX_DATA; i++)
        binary_data_arr[i] = 0;

    // Note: From hereon, we ignore the first (index 0) value. This corresponds to the ~14ms before the actual address & data.
    // Convert duration to binary 0 or 1.
    for (uint32_t i = 1; i < MAX_DATA; i++)
    {
        // Based on the debug code above, any duration above 20000 can be deemed to be a 1.
        if (timer_data_arr[i] > 20000)
            binary_data_arr[i] = 1;
        else
            binary_data_arr[i] = 0;
    }
    
    char * button = get_nec_reading_1(binary_data_arr);
    if (!*button)
        button = get_nec_reading_2(binary_data_arr);

    if (*button)
    {
        char mesg[40 + 1];
        sprintf(mesg, "\n\r%s\n\r", button);
        printString(mesg);
        return(1);
    }
    
    return(0);
}

// Strict checking. First bit is a leading pulse burst (9ms) followed by space (4.5ms). Subsequent 16 bits are address bits (0000000011111111) followed by 16 bits of data.
static char * get_nec_reading_1(const uint32_t binary_data_arr[])
{
    if (TimerValueIdx < 32)
        return("");

    // Expect address 00000000 (bits 1 to 8). We start with bit 1 to skip leading pulse burst and space.
    for (uint32_t i = 1; i < 9; i++)
    {
        if (binary_data_arr[i] != 0)
        {
            return("");
        }
    }
    
    // Expect address 11111111 (bits 9 to 16)
    for (uint32_t i = 9; i < 17; i++)
    {
        if (binary_data_arr[i] != 1)
        {
            return("");
        }
    }
    
    // Expect data (bits 17 to 32)
    uint32_t value = b2i(binary_data_arr, 17);
//    char mesg[40 + 1];
//    sprintf(mesg, "\n\r%d\n\r", value);
//    printString(mesg);
    uint32_t max_data = sizeof(NECData) / sizeof(ir_data_t);
    
    for (uint32_t i = 0; i < max_data; i++)
    {
        if (value == NECData[i].value)
            return(NECData[i].button);
    }
    
    return("");
}

// Sometimes, the initial 8 bits of 00000000 of address are missing or contain less than 8 bits of 0. We cater for this.
// Relaxed checking (look for 8 bit address (11111111) followed by 16 bits of data)
static char * get_nec_reading_2(const uint32_t binary_data_arr[])
{
    // Look for first bit 1 within first 17 bits
    uint32_t j = 0;
    for (uint32_t i = 1; i < 17; i++)
    {
        if (binary_data_arr[i] == 1)
        {
            j = i;
            break;
        }
    }

    if (j == 0)
        return("");
    
    // Expect 11111111 (8 bits)
    for (uint32_t i = j; i < j + 8; i++)
    {
        if (binary_data_arr[i] != 1)
        {
            return("");
        }
    }
    
    // Expect data (16 bits)
    uint32_t value = b2i(binary_data_arr, j + 8);
//    char mesg[40 + 1];
//    sprintf(mesg, "\n\r%d\n\r", value);
//    printString(mesg);
    uint32_t max_data = sizeof(NECData) / sizeof(ir_data_t);
    
    for (uint32_t i = 0; i < max_data; i++)
    {
        if (value == NECData[i].value)
            return(NECData[i].button);
    }
    
    return("");
}

// Convert 16 bits in the passed array starting at the passed postion r to decimal.
static uint32_t b2i(const uint32_t binary_data_arr[], const uint32_t r)
{
    uint32_t result = 0;
    uint32_t j = r + 15;
    uint32_t k = 15;
    for (int32_t i = r; i <= j; i++, k--)
    {
        result = result + (uint32_t) (binary_data_arr[i] * mypow(k));
    }
    return(result);
}

// Don't want to include math.h so roll out own pow() function which assumes base of 2.
static uint32_t mypow(const uint32_t i)
{
    uint32_t result = 1;
    if (i == 0) return(1);
    
    for (uint32_t j = 0; j < i; j++)
    {
        result *= 2;
    }
    
    return(result);
}    
// Set up UART0
static void setup_uart0(void)
{
    //1. Enable the UART module using the RCGCUART register (see page 344)
    UartEnable(UART_0);
    //2. To find out which GPIO port to enable, refer to Table 23-5 on page 1351. UART0 uses port A (U0Rx PA0 Pin 17, U0Tx PA1 Pin 18)
    //   Enable the clock to the appropriate GPIO module via the RCGCGPIO register (see page 340).
    GpioEnable(PORT_A);
    //3. Set the GPIO AFSEL bits 0 and 1 (based on PA0 and PA1)for the appropriate pins (see page 671).
    GPIOA->AFSEL |= (1 << 1) | (1 << 0);
    //4. Configure the GPIO current level and/or slew rate as specified for the mode selected (see page 673 and page 681).
    // Not required
    //5. Configure the PMCn fields in the GPIOPCTL register
    GPIOA->PCTL  |= (1 << 0) | (1 << 4);
    GPIOA->DEN   |= (1 << 0) | (1 << 1);

    // Configure UART0
    // The clock speed used to calculate IBRD/FBRD depends on the UARTCC Register setting of the UART Clock and if the PLL is used or not.
    // If the UARTCC shows PIOSC clock (0x05) as the clock source, then it always is 16MHz and the Baud rate needs to be computed off it (CLOCK_SETUP value has no effect).
    // IBRD=104, FBRD=11
    // If the UARTCC shows the System Clock (0x00) as the clock source, then the RCC-RCC2 registers need to be checked to determine the System Clock and then compute the Baud Rate.
    // CLOCK_SETUP = 1: Use 50MHz system clock, IBRD=325, FBRD=33
    // CLOCK_SETUP = 0: Use 16MHz system clock, IBRD=104, FBRD=11
    // In our code, we are using the 4MHz clock (UARTCC 0x00 with CLOCK_SETUP = 0) 
    UART0->CTL  &= ~(1U << 0);
    UART0->CC   = 0x0;
    UART0->IBRD = 104;
    UART0->FBRD = 11;
    UART0->LCRH = (0x3 << 5);
    UART0->CTL  = (1 << 0) | (1 << 8) | (1 << 9);

    // Configure interrupt.
    // Set Interrupt Clear Register (Receive Interrupt Clear)
    // Set Interrupt Mask Register
    // Set Interrupt Set Enable Register through NVIC i.e. the interrupt must be enabled in the NVIC_ENm_R register.
    // Each NVIC_ENm_R register has 32-bits, and each bit controls one interrupt number. Using the following formula to find out NVIC_ENm_R register number
    // and the bit number to enable the interrupt on NVIC.
    // m = interrupt number / 32
    // b = interrupt number % 32
    // NVIC->ISER[m] |= (1UL << b);
    // For UART0, the interrupt number is 5 (see startup_TM4C123.s; look for UART0_Handler; you will find "5: UART0 Rx and Tx")
    // m = 5 / 32 = 0; hence we update ISER[0]; b = 5 % 32 = 5; hence we set bit 5.
    UART0->ICR &= ~(1U << 4);
    UART0->IM |= (1 << 4);
    NVIC->ISER[0] |= (1 << 5);
}

static void printString(const char * string)
{
    while (*string)
    {
        printChar(*(string++));
    }
}

static void printChar(const char c)
{
    while ((UART0->FR & (1 << 5)) != 0);
    UART0->DR = c;
}

// We are using WTIMER0A in capture mode to capture the echo from the sensor. 
// We are using GPIO PC4 alternate function for the capture.
// Pin Name: WT0CCP0, Pin Number: 16, Pin Mux/Pin Assignment: PC4 (7), Pin Type: I/O, Buffer Type: TTL, Description: 32/64-Bit Timer 0 Capture/Compare/PWM 0.
// See Table 11-2.
static void sensor_input(void)
{
    // Disable clock gating for WTIMER module.
    SYSCTL->RCGCWTIMER |= (1 << 0);  // WTIMER0
    // There must be a delay of 3 system clocks after a peripheral module clock is enabled in the RCGC register
    // before any module registers are accessed. See page 227 (System Control). We check PRWTIMER.
    while (1)
    {
        if (SYSCTL->PRWTIMER & (1 << 0))
            break;
    }

    // Configure GPIO PC4.
    GpioEnable(PORT_C);
    GPIOC->DEN |= (1 << 4);
    GPIOC->DIR &= ~(1U << 4);
    GPIOC->AFSEL |= (1 << 4);
    GPIOC->PCTL &= ~0x00070000U;  // see Table 11-2 for value (7 for PC4/WT0CCP0) and Register GPIOPCTL for bit position (19:16 PMC4 ...)
    GPIOC->PCTL |= 0x00070000;

//1. Ensure the timer is disabled (the TnEN bit is cleared) before making any changes.
    WTIMER0->CTL &= ~(1U << 0);
    
//2. Write the GPTM Configuration (GPTMCFG) register with a value of 0x0000.0004.
    WTIMER0->CFG = 0x4;              // 32-bit timer
    
//3. In the GPTM Timer Mode (GPTMTnMR) register, set TnCDIR bit to 1, TnAMS bit to 0, the TnCMR bit to
//0x1, and the TnMR field to 0x3.
    WTIMER0->TAMR = 0x17;

//4. Configure the type of event that the timer captures by writing the TnEVENT field of the GPTM
//Control (GPTMCTL) register.
    WTIMER0->CTL |= 0x04;      // TAEVENT Negative edge
    
//5. If a prescaler is to be used, write the prescale value to the GPTM Timer n Prescale Register
//(GPTMTnPR).
    // Nothing to do
//6. Load the timer start value into the GPTM Timer n Interval Load (GPTMTnILR) register.
   // Nothing to do    
//7. If interrupts are required, set the CnEIM bit in the GPTM Interrupt Mask (GPTMIMR) register.
    WTIMER0->ICR = (1 << 2);     // clear interrupt
    WTIMER0->IMR = (1 << 2);

//  Initialise timer to always start from 0.
    WTIMER0->TAV = 0;

//8. Set the TnEN bit in the GPTM Control (GPTMCTL) register to enable the timer and start counting.
    WTIMER0->CTL |= (1 << 0);
    
//9. Poll the CnERIS bit in the GPTMRIS register or wait for the interrupt to be generated (if enabled).
//In both cases, the status flags are cleared by writing a 1 to the CnECINT bit of the GPTM
//Interrupt Clear (GPTMICR) register. The time at which the event happened can be obtained
//by reading the GPTM Timer n (GPTMTnR) register.
    // See WTIMER0A_Handler().

    // Set Interrupt Set Enable Register through NVIC i.e. the interrupt must be enabled in the NVIC_ENm_R register.
    // Each NVIC_ENm_R register has 32-bits, and each bit controls one interrupt number. Using the following formula to find out NVIC_ENm_R register number
    // and the bit number to enable the interrupt on NVIC.
    // m = interrupt number / 32
    // b = interrupt number % 32
    // NVIC->ISER[m] |= (1UL << b);
    // For WTIMER0A, the interrupt number is 94 (see startup_TM4C123.s; look for WTIMER0A_Handler; you will find "94: WTIMER0A ...")
    // m = 94 / 32 = 2; hence we update ISER[2]; b = 94 % 32 = 30; hence we set bit 30.
    NVIC->ISER[2] |= (1 << 30);
}

void WTIMER0A_Handler(void)
{
    WTIMER0->ICR = (1 << 2);
    if (TimerValueIdx < MAX_DATA)
        TimerValueArr[TimerValueIdx++] = WTIMER0->TAR;
}

static void setup_systick(void)
{
//Delay time calculation (assume 16MHz):
//Since we are working with external clock i.e. 16 MHz, each pulse generated by the clock source will have:
//1/XTAL frequency = 1/(16*10^6) = 62.5ns time period. 
//So if we load 253 into the RELOAD register and trigger the counter it will count down with next pulse and will take 62.5ns to change its value from 253 to 252.
//Hence, In order to generate the delay, we can calculate the approximate value that has to be loaded into this register by the formula-
//Reload Value = XTAL*Time delay
//one extra clock delay is already included to set the flag for rollover, hence we get one extra clock delay. By subtracting by one will give exact time delay.
//Reload Value = (XTAL*Time Delay)-1
//Remember that in one shoot, it can only take 0xFFFFFF maximum value (24 bits). Therefore in one shoot, we can only generate maximum of Time delay
//TimeDelay = (ReloadValue+1)/XTAL=16777215+(1/(16*10^6))
//TimeDelay = 1.048575937 sec.
//Example: For generating 1us (0.000001 second) delay using 16MHz, the value that has to be loaded into the RELOAD Register
//Reload Value = (XTAL*Time delay)-1
//Reload Value = (16*10^6*.000001)-1
//Reload Value = 15
//Example: For generating 1ms (0.001 second) delay using 16MHz, the value that has to be loaded into the RELOAD Register
//Reload Value = (XTAL*Time delay)-1
//Reload Value = (16*10^6*.001)-1
//Reload Value = 15999
//Example: For generating 1 second delay using 16MHz, the value that has to be loaded into the RELOAD Register
//Reload Value = (XTAL*Time delay)-1
//Reload Value = (16*10^6*1)-1
//Reload Value = 15999999
        
    // We will generate 1ms time delay using 16MHz (CLOCK_SETUP is 0)
    // Reload value = (16*10^6*.001)-1 = 15999
    // Configure clock. See Tiva C DS page 123 (System Timer). For configuring CTRL see page 138 (Register 1);
    // We need to set bits 0, 1, 2 i.e. 0111 == 07U or (1U << 2) | (1U << 1) | (1U)
    SysTick->CTRL = 0;
    SysTick->LOAD = 15999;  // Cannot exceed 24 bits i.e. 16777215
    SysTick->VAL  = 0U;
    SysTick->CTRL = (1 << 2) | (1 << 1) | (1 << 0);
}

void SysTick_Handler(void)
{
    CurrentTicks++;
}


