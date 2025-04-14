//#############################################################################
// FILE:   HWFinal_main.c
//
// TITLE:  ACHWFinalProject
//#############################################################################

// Included Files
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "F28379dSerial.h"
#include "LEDPatterns.h"
#include "song.h"
#include "dsp.h"
#include "fpu32/fpu_rfft.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
// The Launchpad's CPU Frequency set to 200 you should not change this value
#define LAUNCHPAD_CPU_FREQUENCY 200

// AC Define center voltage for joystick
#define centerVoltage1 1.69482422
#define centerVoltage2 1.61499023
// AC Define min and max speeds
#define minSpeed 0.0 // To be determined
#define maxSpeed 0.25 // To be determined

// AC Define notes for song
#define C4_NOTE 47778 // AC C4 is 261.63 Hz so 25 MHz / (2 * 261.63) = 47778
#define D4_NOTE 42566
#define E4_NOTE 37922
#define F4_NOTE 35789
#define G4_NOTE 31888
#define A4_NOTE 28409
#define FS4_NOTE 33860  // 25MHz / (2 * 369.99 Hz)
#define GS4_NOTE 30189  // 25MHz / (2 * 415.30 Hz)
#define CS5_NOTE 22606  // 25MHz / (2 * 554.37 Hz)
#define B4_NOTE 25252   // 25MHz / (2 * 493.88 Hz)
#define CS4_NOTE 45018  // 25MHz / (2 * 277.18 Hz)
#define E5_NOTE 18958  // 25MHz / (2 * 659.25 Hz)
#define D5_NOTE 21204  // 25MHz / (2 * 587.33 Hz)

#define nokiaLength 25  // AC

uint16_t nokiaMelody[nokiaLength] = {
                                     E5_NOTE, E5_NOTE, D5_NOTE, D5_NOTE,
                                     FS4_NOTE, FS4_NOTE, GS4_NOTE, GS4_NOTE,
                                     CS5_NOTE, CS5_NOTE, B4_NOTE, B4_NOTE,
                                     D4_NOTE, D4_NOTE, E4_NOTE, E4_NOTE,
                                     B4_NOTE, B4_NOTE, A4_NOTE, A4_NOTE,
                                     CS4_NOTE, CS4_NOTE, E4_NOTE, E4_NOTE,
                                     A4_NOTE, A4_NOTE
};


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);

// AC intialize ADCA_ISR function
__interrupt void ADCA_ISR(void);


// AC Predefine RC Servo functions
void setEPWM8A_RCServo(float angle);
void setEPWM8B_RCServo(float angle);


// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;

// AC variables to setup adca for joystick
int32_t adca_count = 0;   // AC count variable for ADCA_ISR func
int32_t Adca2result = 0;  // AC raw results of joystick y-axis ADCINA2 input
int32_t Adca3result = 0;  // AC raw results of joystick x-axis ADCINA3 input
float ADCINA2_volt = 0.0; // AC volt from joystick y-axis ADCINA2 input
float ADCINA3_volt = 0.0; // AC volt from joystick x-axis ADCINA3 input


// AC Define RC motor counters
int16_t motorCounter = 0;
float hingeAngle1 = 0.0;
float hingeAngle2 = 0.0;
float deviation1 = 0.0;
float deviation2 = 0.0;
float speed_factor1 = 0.0;
float step_size1 = 0.0;
float speed_factor2 = 0.0;
float step_size2 = 0.0;
int32_t soundCount = 0;

void main(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();

    // AC Set buzzer as PWM
    GPIO_SetupPinMux(16, GPIO_MUX_CPU1, 5); // Set GPIO16 as GPIO instead of PWM
    //    GPIO_SetupPinMux(16, GPIO_MUX_CPU1, 1); // Set GPIO16 as GPIO as not PWM for exercises 4 and 5
    GPIO_SetupPinOptions(16, GPIO_OUTPUT, GPIO_PUSHPULL); // AC configure as output
    GpioDataRegs.GPASET.bit.GPIO16 = 1; // AC Set low

    // Blue LED on LaunchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

    // Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

    // LED1 and PWM Pin
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;

    // LED2
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

    // LED3
    GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;

    // LED4
    GPIO_SetupPinMux(97, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(97, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

    // LED5
    GPIO_SetupPinMux(111, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(111, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

    // LED6
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

    // LED7
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

    // LED8
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

    // LED9
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

    // LED10
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

    // LED11
    GPIO_SetupPinMux(60, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(60, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;

    // LED12
    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

    // LED13
    GPIO_SetupPinMux(157, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(157, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;

    // LED14
    GPIO_SetupPinMux(158, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(158, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;

    // LED15
    GPIO_SetupPinMux(159, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(159, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO159 = 1;

    // LED16
    GPIO_SetupPinMux(160, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(160, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPFCLEAR.bit.GPIO160 = 1;

    //WIZNET Reset
    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO0 = 1;

    //ESP8266 Reset
    GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(1, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO1 = 1;

    //SPIRAM  CS  Chip Select
    GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;

    //DRV8874 #1 DIR  Direction
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO29 = 1;

    //DRV8874 #2 DIR  Direction
    GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(32, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO32 = 1;

    //DAN28027  CS  Chip Select
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO9 = 1;

    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;

    //WIZNET  CS  Chip Select
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDSET.bit.GPIO125 = 1;

    //PushButton 1
    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(4, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 2
    GPIO_SetupPinMux(5, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(5, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 3
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(6, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 4
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(7, GPIO_INPUT, GPIO_PULLUP);

    //Joy Stick Pushbutton
    GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(8, GPIO_INPUT, GPIO_PULLUP);

    // RC servo motor (EPWM8A)
    GPIO_SetupPinMux(14, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinOptions(14, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPFCLEAR.bit.GPIO160 = 1;


    // RC servo motor (EPWM8B)
    GPIO_SetupPinMux(15, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinOptions(15, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPFCLEAR.bit.GPIO160 = 1;

    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
    PieVectTable.SCIB_RX_INT = &RXBINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIB_TX_INT = &TXBINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;

    // AC add to map ADCA1_INT to ADCA_ISR func
    PieVectTable.ADCA1_INT = &ADCA_ISR;

    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 10000);
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 20000);
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 40000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serialSCIA(&SerialA,115200);
    //    init_serialSCIC(&SerialC,115200);
    //    init_serialSCID(&SerialD,115200);

    // AC four lines: Count up, Free Soft, Free run and clock divide by 1
    EPwm8Regs.TBCTL.bit.CTRMODE = 0; //AC set count up mode
    EPwm8Regs.TBCTL.bit.FREE_SOFT = 2; // AC Free run
    EPwm8Regs.TBCTL.bit.PHSEN = 0; // AC Disable phase loading
    EPwm8Regs.TBCTL.bit.CLKDIV = 4; // AC Clock divide by 16 (100 for 16 in the data sheet) so 50 MHz/16 = 3.125 MHz




    // AC set time based counter to 0
    EPwm8Regs.TBCTR = 0;




    // AC Set period for 5 kHz
    EPwm8Regs.TBPRD = 62500; //AC 3125000 Hz / 50 Hz = 62500 (max TBPRD is 65535)




    // AC Set Compare A register to 50% duty cycle
    EPwm8Regs.CMPA.bit.CMPA = 5000; // AC 62500 * 8% = 5000
    EPwm8Regs.CMPB.bit.CMPB = 5000; // AC 62500 * 8% = 5000






    EPwm8Regs.AQCTLA.bit.CAU = 1; //AC set PWM output high when TBCTR = 0
    EPwm8Regs.AQCTLA.bit.ZRO = 2; //AC clear PWM output low when TBCTR = CMPA




    EPwm8Regs.AQCTLB.bit.CBU = 1; //AC set PWM output high when TBCTR = 0
    EPwm8Regs.AQCTLB.bit.ZRO = 2; //AC clear PWM output low when TBCTR = CMPB




    EPwm8Regs.TBPHS.bit.TBPHS = 0; // AC Set phase zero

    // AC four lines: Count up, Free Soft, Free run and clock divide by 1
    EPwm9Regs.TBCTL.bit.CTRMODE = 0; //AC set count up mode
    EPwm9Regs.TBCTL.bit.FREE_SOFT = 2; // AC Free run
    EPwm9Regs.TBCTL.bit.PHSEN = 0; // AC Disable phase loading
    EPwm9Regs.TBCTL.bit.CLKDIV = 1; // AC Clock divide by 2


    // AC set time based counter to 0
    EPwm9Regs.TBCTR = 0;


    // AC Set period for 5 kHz
    EPwm9Regs.TBPRD = 10000; //AC 50 MHz / 5 kHz = 10000


    // AC Set Compare A register to 50% duty cycle
    //EPwm9Regs.CMPA.bit.CMPA = 5000; // AC 10000/2 = 5000


    EPwm9Regs.AQCTLA.bit.CAU = 0;  // AC Do nothing when CMPA is reached (since we're not using CMPA)
    EPwm9Regs.AQCTLA.bit.ZRO = 3;  // AC Toggle the output when the counter reaches zero


    EPwm9Regs.TBPHS.bit.TBPHS = 0; // AC Set phase zero


    //AC first code paragraph from HW doc
    EALLOW;
    EPwm4Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    EPwm4Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
    EPwm4Regs.ETSEL.bit.SOCASEL = 2; // Select Event when counter equal to PRD
    EPwm4Regs.ETPS.bit.SOCAPRD = 1; // Generate pulse on 1st event (“pulse” is the same as “trigger”)
    EPwm4Regs.TBCTR = 0x0; // Clear counter
    EPwm4Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm4Regs.TBCTL.bit.PHSEN = 0; // Disable phase loading
    EPwm4Regs.TBCTL.bit.CLKDIV = 0; // divide by 1 50Mhz Clock
    EPwm4Regs.TBPRD = 50000; // Set Period to 1ms sample. Input clock is 50MHz.
    // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal
    EPwm4Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
    EPwm4Regs.TBCTL.bit.CTRMODE = 0;//unfreeze, and enter up count mode
    EDIS;


    // AC second paragraph of code from HW
    EALLOW;
    // Write configurations for ADCA
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; // Set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); // Read calibration settings




    // Set pulse positions to late
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;




    // Power up the ADCs
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;




    // Delay for 1ms to allow ADC time to power up
    DELAY_US(1000);


    // Select the channels to convert and end of conversion flag
    // ADCA
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 4; // SOC0 will convert Channel ADCINA4
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 99; // AC Sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 11; // AC SOC0 will begin conversion on EPWM4 ADCSOCA


    // AC joystick setup
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 2; // AC SOC1 -> ADCINA2 (Joystick X)
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 99; // AC Sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 11; // AC EPWM4




    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 3; // AC SOC2 -> ADCINA3 (Joystick Y)
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = 99; // AC Sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 11; // AC EPWM4


    // AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; // AC Set to last or only SOC that is converted and it will set INT1 flag ADCA1
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 2;  // AC Set INT1 to trigger after SOC2 (Joystick Y)
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;  // AC Enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // AC Make sure INT1 flag is cleared
    EDIS;


    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;

    // AC Enable ADCA1 in the PIE: Group 1, Channel 1
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;

    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM


    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
            serial_printf(&SerialA,"Num Timer2:%ld Num SerialRX: %ld\r\n",CpuTimer2.InterruptCount,numRXA);
            UARTPrint = 0;
        }
    }
}


// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void) {

    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
    // making it lower priority than all other Hardware interrupts.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");                    // Wait one cycle
    EINT;                                 // Clear INTM to enable interrupts



    // Insert SWI ISR Code here.......


    numSWIcalls++;

    DINT;

}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    numTimer0calls++;

    //    if ((numTimer0calls%50) == 0) {
    //        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
    //    }
    //
    //    if ((numTimer0calls%250) == 0) {
    //        displayLEDletter(LEDdisplaynum);
    //        LEDdisplaynum++;
    //        if (LEDdisplaynum == 0xFFFF) {  // prevent roll over exception
    //            LEDdisplaynum = 0;
    //        }
    //    }

    // Blink LaunchPad Red LED
    GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;

    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{


    CpuTimer1.InterruptCount++;
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{


    // Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    CpuTimer2.InterruptCount++;

    if ((CpuTimer2.InterruptCount % 50) == 0) {
        UARTPrint = 1;
    }
}

__interrupt void ADCA_ISR (void)
{
    // AC set output of Adca's to variables
    Adca2result = AdcaResultRegs.ADCRESULT1;
    Adca3result = AdcaResultRegs.ADCRESULT2;




    // AC scale raw joystick integer to float volatge
    ADCINA2_volt = 3.0 * Adca2result/4096.0; // AC (joystick y)
    ADCINA3_volt = 3.0 * Adca3result/4096.0; // AC (joystick x)


    // AC joystick logic for hinges


    // *****************************//

    // AC First hinge
    // AC Calculate speed based on deviation from center
    deviation1 = (ADCINA2_volt - centerVoltage1);  // AC 0 to 1.6
    speed_factor1 = deviation1 / centerVoltage1;   // AC -1 to 1
    step_size1 = minSpeed + (speed_factor1 * (maxSpeed - minSpeed));

    // AC zero out step size if Joystick is aproximatley in center
    if (step_size1 > -0.005 && step_size1 < 0.005) {
        step_size1 = 0.0;
    }




    // AC code to saturate hinge angle here
    if (hingeAngle1 > 90.0) {
        hingeAngle1 = 90.0;
    }
    if (hingeAngle1 < -90.0) {
        hingeAngle1 = -90.0;
    }



    // AC update motor
    if (adca_count % 1000) {
        hingeAngle1 += step_size1;
    }
    setEPWM8A_RCServo(hingeAngle1);

    // AC Second hinge
    // AC Calculate speed based on deviation from center
    deviation2 = (ADCINA3_volt - centerVoltage2);  // AC 0 to 1.6
    speed_factor2 = deviation2 / centerVoltage2;   // AC -1 to 1
    step_size2 = minSpeed + (speed_factor2 * (maxSpeed - minSpeed));

    // AC zero out step size if Joystick is aproximatley in center
    if (step_size2 > -0.005 && step_size2 < 0.005) {
        step_size2 = 0.0;
    }




    // AC code to saturate hinge angle here
    if (hingeAngle2 > 10.0) {
        hingeAngle2 = 10.0;
    }
    if (hingeAngle2 < -90.0) {
        hingeAngle2 = -90.0;
    }



    // AC update motor
    if (adca_count % 1000) {
        hingeAngle2 += step_size2;
    }
    setEPWM8B_RCServo(-hingeAngle2);

    // *****************************//

    //    hingeAngle1 = (180.0/3.0)* ADCINA2_volt - 90;
    //    setEPWM8A_RCServo(hingeAngle1);



    if(hingeAngle2 == -90) {
        if (soundCount < nokiaLength) {
            // AC Set buzzer as PWM
            GPIO_SetupPinMux(16, GPIO_MUX_CPU1, 5); // Set GPIO16 as GPIO instead of PWM
            GPIO_SetupPinOptions(16, GPIO_OUTPUT, GPIO_PUSHPULL); // AC configure as output
            GpioDataRegs.GPASET.bit.GPIO16 = 1; // AC Set low
//            EPwm9Regs.TBPRD = E5_NOTE;
        }

        else {
            GPIO_SetupPinMux(16, GPIO_MUX_CPU1, 0); // Set GPIO16 as GPIO instead of PWM
            GPIO_SetupPinOptions(16, GPIO_OUTPUT, GPIO_PUSHPULL); // AC configure as output
            GpioDataRegs.GPASET.bit.GPIO16 = 1; // AC Set low
            soundCount = 0;
        }
        if ((adca_count % 125) == 0) {
            EPwm9Regs.TBPRD = nokiaMelody[soundCount]; // AC change the period to period of song notes
            soundCount++;
        }

    }
    else {
                GPIO_SetupPinMux(16, GPIO_MUX_CPU1, 0); // Set GPIO16 as GPIO instead of PWM
                GPIO_SetupPinOptions(16, GPIO_OUTPUT, GPIO_PUSHPULL); // AC configure as output
                GpioDataRegs.GPASET.bit.GPIO16 = 1; // AC Set low
                soundCount = 0;
            }

    // AC increment counter
     adca_count++;


    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}




void setEPWM8A_RCServo(float angle) {
    // Saturate at 90
    if (angle > 90) {
        angle = 90;
    }
    // Saturate at -90
    if (angle < -90) {
        angle = -90;
    }




    // AC Convert angle to CMPA reading
    // AC 90 -> 62500 * 12% = 7500 and -90 -> 62500 * 4% = 2500
    EPwm8Regs.CMPA.bit.CMPA = ((7500 - 2500)/(90 - (-90))) * angle + 5000;
}




void setEPWM8B_RCServo(float angle) {
    // Saturate at 90
    if (angle > 90) {
        angle = 90;
    }
    // Saturate at -90
    if (angle < -90) {
        angle = -90;
    }




    // AC Convert angle to CMPB reading
    // AC 90 -> 62500 * 12% = 7500 and -90 -> 62500 * 4% = 2500
    EPwm8Regs.CMPB.bit.CMPB = ((7500 - 2500)/(90 - (-90))) * angle + 5000;




}
