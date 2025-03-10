// Alan Connor is commenting this code
//#############################################################################
// FILE:   HWstarter_main.c
//
// TITLE:  HW Starter
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


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);

// AC intialize ADCA_ISR function
__interrupt void ADCA_ISR (void);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;

int16_t updown = 0; // AC toggle LED brightness up or down
int16_t counter = 1; // AC count variables for LED
int32_t adca_count = 0; // AC count variable for ADCA_ISR func
int32_t Adca4result = 0; // AC raw results of photo resistor ADCINA4 input
int32_t Adca2result = 0; // AC raw results of joystick y-axis ADCINA2 input
int32_t Adca3result = 0; // AC raw results of joystick x-axis ADCINA3 input
float ADCINA4_volt = 0.0; // AC volt from photo resistor ADCINA4 input
float ADCINA2_volt = 0.0; // AC volt from joystick y-axis ADCINA2 input
float ADCINA3_volt = 0.0; // AC volt from joystick x-axis ADCINA3 input


void main(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();

    // Blue LED on LaunchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

    // Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

    // LED1 and PWM Pin
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0); // AC change last parameter to 5 for exercises 1 to 3 and to 0 for exercise 4
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
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 1000); // AC set period to 1000 useconds or 1 msecond

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serialSCIA(&SerialA,115200);
    //    init_serialSCIC(&SerialC,115200);
    //    init_serialSCID(&SerialD,115200);

    // AC four lines: Count up, Free Soft, Free run and clock divide by 1
    EPwm12Regs.TBCTL.bit.CTRMODE = 0; //AC set count up mode
    EPwm12Regs.TBCTL.bit.FREE_SOFT = 2; // AC Free run
    EPwm12Regs.TBCTL.bit.PHSEN = 0; // AC Disable phase loading
    EPwm12Regs.TBCTL.bit.CLKDIV = 0; // AC Clock divide by 1

    // AC set time based counter to 0
    EPwm12Regs.TBCTR = 0;

    // AC Set period for 20 kHz
    EPwm12Regs.TBPRD = 10000; //AC 50 MHz / 5 kHz = 10000

    // AC Set Compare A register to 50% duty cycle
//    EPwm12Regs.CMPA.bit.CMPA = 5000; // AC 10000/2 = 5000

    EPwm12Regs.AQCTLA.bit.CAU = 1; //AC set PWM output high when TBCTR = 0
    EPwm12Regs.AQCTLA.bit.ZRO = 2; //AC clear PWM output low when TBCTR = CMPA

    EPwm12Regs.TBPHS.bit.TBPHS = 0; // AC Set phase zero

    // AC disable pull-up resistor
    EALLOW; // Below are protected registers
    GpioCtrlRegs.GPAPUD.bit.GPIO22 = 1; // For EPWM12A
    EDIS;

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
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 99; // Sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 11; // SOC0 will begin conversion on EPWM4 ADCSOCA


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
            //serial_printf(&SerialA,"Num Timer2:%ld Num SerialRX: %ld\r\n",CpuTimer2.InterruptCount,numRXA);
            // AC changed print to display photo resistor and joystick voltage
            serial_printf(&SerialA, "Photo Resistor Voltage: %.3f Joystick y-axis voltage: %.3f Joystick x-axis voltage: %.3f \r\n", ADCINA4_volt, ADCINA2_volt, ADCINA3_volt);
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

    // AC increment counter every msecond until the value 10000 then decrement it until 0 then repeat
    if(updown ==1){
        counter = counter +1;
        if(counter == 10000){
            updown = 0;
        }
    }
    else{
        counter = counter -1;
        if(counter == 0){
            updown = 1;
        }
    }

    // AC set CMPA to value of counter
    //EPwm12Regs.CMPA.bit.CMPA = counter;



    // Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    CpuTimer2.InterruptCount++;

//    if ((CpuTimer2.InterruptCount % 50) == 0) {
//        UARTPrint = 1;
//    }
}


__interrupt void ADCA_ISR (void)
{
   // AC set output of Adca's to variables
   Adca4result = AdcaResultRegs.ADCRESULT0;
   Adca2result = AdcaResultRegs.ADCRESULT1;
   Adca3result = AdcaResultRegs.ADCRESULT2;




   // AC scale raw photo resistor integer to float volatge
   ADCINA4_volt = 3.0 * Adca4result/4096.0;
   ADCINA2_volt = 3.0 * Adca2result/4096.0;
   ADCINA3_volt = 3.0 * Adca3result/4096.0;


   // AC set LED to correlate with photo resistor input
//   EPwm12Regs.CMPA.bit.CMPA = Adca4result * 10000.0/4096.0;


   // AC joystick LED pattern


   // AC Turn off all LEDs at the beginning
   GpioDataRegs.GPACLEAR.all = 0xFFFFFFFF; // AC Clear all GPIOA LEDs
   GpioDataRegs.GPBCLEAR.all = 0xFFFFFFFF; // AC Clear all GPIOB LEDs
   GpioDataRegs.GPCCLEAR.all = 0xFFFFFFFF; // AC Clear all GPIOC LEDs
   GpioDataRegs.GPDCLEAR.all = 0xFFFFFFFF; // AC Clear all GPIOD LEDs
   GpioDataRegs.GPECLEAR.all = 0xFFFFFFFF; // AC Clear all GPIOE LEDs
   GpioDataRegs.GPFCLEAR.all = 0xFFFFFFFF; // AC Clear all GPIOF LEDs


   // AC y-axis of joystick first if and nested if for x-axis
   if (ADCINA2_volt < 1) {
         if (ADCINA3_volt < 0.6) {
           GpioDataRegs.GPESET.bit.GPIO159 = 1; // AC turn on LED 15
       } else if (ADCINA3_volt < 1.2) {
           GpioDataRegs.GPESET.bit.GPIO158 = 1; // AC turn on LED 14
       } else if (ADCINA3_volt < 1.8) {
           GpioDataRegs.GPESET.bit.GPIO157 = 1; // AC turn on center (LED 13)
       } else if (ADCINA3_volt < 2.4) {
           GpioDataRegs.GPBSET.bit.GPIO61 = 1; // AC turn on LED 12
       } else {
           GpioDataRegs.GPBSET.bit.GPIO60 = 1; // AC turn on LED 11
       }
   } else if (ADCINA2_volt < 2) {
       if (ADCINA3_volt < 0.6) {
           GpioDataRegs.GPASET.bit.GPIO27 = 1; // AC turn on LED 10
       } else if (ADCINA3_volt < 1.2) {
           GpioDataRegs.GPASET.bit.GPIO26 = 1; // AC turn on LED 9
       } else if (ADCINA3_volt < 1.8) {
           GpioDataRegs.GPASET.bit.GPIO25 = 1; // AC turn on center (LED 8)
       } else if (ADCINA3_volt < 2.4) {
           GpioDataRegs.GPESET.bit.GPIO131 = 1; // AC turn on LED 7
       } else {
           GpioDataRegs.GPESET.bit.GPIO130 = 1; // AC turn on LED 6
       }
   } else  {
       if (ADCINA3_volt < 0.6) {
           GpioDataRegs.GPDSET.bit.GPIO111 = 1; // AC turn on LED 5
       } else if (ADCINA3_volt < 1.2) {
           GpioDataRegs.GPDSET.bit.GPIO97 = 1;// AC turn on LED 4
       } else if (ADCINA3_volt < 1.8) {
           GpioDataRegs.GPCSET.bit.GPIO95 = 1; // AC turn on middle (LED 3)
       } else if (ADCINA3_volt < 2.4) {
           GpioDataRegs.GPCSET.bit.GPIO94 = 1; // AC turn on LED 2
       } else {
           GpioDataRegs.GPASET.bit.GPIO22 = 1; // AC turn on LED 1
       }
   }




   // AC increment counter
   adca_count++;




   // AC ADCA_ISR func is called every 1ms so print every 100th time or 100ms
   if ((adca_count % 100) == 0) {
       UARTPrint = 1;
   }






   AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


