//#############################################################################
// FILE:   HWstarter_AC_main.c
//
// TITLE:  ACHW3
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

#define lilStarLength 56 // Updated length

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


uint16_t lilStar[lilStarLength] = {
                                   // First line: C4, C4, G4, G4, A4, A4, G4 (2 beats)
                                   C4_NOTE, C4_NOTE, // C4 (1 beat, repeated 2 times)
                                   C4_NOTE, C4_NOTE, // C4 (1 beat, repeated 2 times)
                                   G4_NOTE, G4_NOTE, // G4 (1 beat, repeated 2 times)
                                   G4_NOTE, G4_NOTE, // G4 (1 beat, repeated 2 times)
                                   A4_NOTE, A4_NOTE, // A4 (1 beat, repeated 2 times)
                                   A4_NOTE, A4_NOTE, // A4 (1 beat, repeated 2 times)
                                   G4_NOTE, G4_NOTE, G4_NOTE, G4_NOTE, // G4 (2 beats, repeated 4 times)


                                   // Second line: F4, F4, E4, E4, D4, D4, C4 (2 beats)
                                   F4_NOTE, F4_NOTE, // F4 (1 beat, repeated 2 times)
                                   F4_NOTE, F4_NOTE, // F4 (1 beat, repeated 2 times)
                                   E4_NOTE, E4_NOTE, // E4 (1 beat, repeated 2 times)
                                   E4_NOTE, E4_NOTE, // E4 (1 beat, repeated 2 times)
                                   D4_NOTE, D4_NOTE, // D4 (1 beat, repeated 2 times)
                                   D4_NOTE, D4_NOTE, // D4 (1 beat, repeated 2 times)
                                   C4_NOTE, C4_NOTE, C4_NOTE, C4_NOTE, // C4 (2 beats, repeated 4 times)


                                   // Third line: G4, G4, F4, F4, E4, E4, D4 (2 beats)
                                   G4_NOTE, G4_NOTE, // G4 (1 beat, repeated 2 times)
                                   G4_NOTE, G4_NOTE, // G4 (1 beat, repeated 2 times)
                                   F4_NOTE, F4_NOTE, // F4 (1 beat, repeated 2 times)
                                   F4_NOTE, F4_NOTE, // F4 (1 beat, repeated 2 times)
                                   E4_NOTE, E4_NOTE, // E4 (1 beat, repeated 2 times)
                                   E4_NOTE, E4_NOTE, // E4 (1 beat, repeated 2 times)
                                   D4_NOTE, D4_NOTE, D4_NOTE, D4_NOTE, // D4 (2 beats, repeated 4 times)


                                   // Fourth line: G4, G4, F4, F4, E4, E4, D4 (2 beats)
                                   G4_NOTE, G4_NOTE, // G4 (1 beat, repeated 2 times)
                                   G4_NOTE, G4_NOTE, // G4 (1 beat, repeated 2 times)
                                   F4_NOTE, F4_NOTE, // F4 (1 beat, repeated 2 times)
                                   F4_NOTE, F4_NOTE, // F4 (1 beat, repeated 2 times)
                                   E4_NOTE, E4_NOTE, // E4 (1 beat, repeated 2 times)
                                   E4_NOTE, E4_NOTE, // E4 (1 beat, repeated 2 times)
                                   D4_NOTE, D4_NOTE, D4_NOTE, D4_NOTE  // D4 (2 beats, repeated 4 times)
};



// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);

// AC predifne SPIB_isr func
__interrupt void SPIB_isr(void);

// AC predefine the setupSpib func
void setupSpib(void);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;

// AC define variables for exercise 4
int16_t spivalue1 = 0;
int16_t spivalue2 = 0;
int16_t gyroz_raw = 0; // AC Raw Gyro Z value
uint32_t spib_counter = 0; // AC Counter for timing

// AC Gyro and Acceleraometer Variables
int16_t accel_x_raw = 0, accel_y_raw = 0, accel_z_raw = 0;
int16_t gyro_x_raw = 0, gyro_y_raw = 0, gyro_z_raw = 0;
float accel_x_g = 0, accel_y_g = 0, accel_z_g = 0;
float gyro_x_dps = 0, gyro_y_dps = 0, gyro_z_dps = 0;
int16_t spi_data_ready = 0; // Flag to indicate new data is ready


void main(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();

    // AC Set buzzer as PWM
    //    GPIO_SetupPinMux(16, GPIO_MUX_CPU1, 5); // Set GPIO16 as GPIO instead of PWM
    GPIO_SetupPinMux(16, GPIO_MUX_CPU1, 1); // Set GPIO16 as GPIO as not PWM for exercises 4 and 5
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

    PieVectTable.SPIB_RX_INT = &SPIB_isr; // AC add func to pie vect table
    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 1000); // AC call timer 0 every 1000 uSeconds or 1 mSecond
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 125000); // AC call timer every  125000 uSeconds or 125 mSeconds
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 40000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serialSCIA(&SerialA,115200);
    //    init_serialSCIC(&SerialC,115200);
    //    init_serialSCID(&SerialD,115200);

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


    // AC disable pull-up resistor
    EALLOW; // Below are protected registers
    GpioCtrlRegs.GPAPUD.bit.GPIO22 = 1; // For EPWM12A
    EDIS;


    setupSpib(); // AC Call setupSpib function

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;
    IER |= M_INT6;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
    // AC Enable SPIB_RX interrupt in PIE Group 6
    PieCtrlRegs.PIEIER6.bit.INTx3 = 1;  // AC Enable SPIB_RX interrupt (INT6.3)

    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM


    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
            //serial_printf(&SerialA, "Gyro Z Raw: %d\r\n", gyroz_raw); // AC Exercise 4 print raw gyro
            // AC print accelerometer and gyro values in g and dps for exercise 5
            serial_printf(&SerialA, "Accel X: %.2f, Y: %.2f, Z: %.2f, Gyro X: %.2f, Y: %.2f, Z: %.2f\r\n", accel_x_g, accel_y_g, accel_z_g, gyro_x_dps, gyro_y_dps, gyro_z_dps);
            //            serial_printf(&SerialA, "Gyro X: %.2f, Y: %.2f, Z: %.2f\r\n", gyro_x_dps, gyro_y_dps, gyro_z_dps);
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

    //Clear GPIO66 Low to act as a Slave Select. Right now, just to scope. Later to select MPU9250 chip
    //    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    //    SpibRegs.SPIFFRX.bit.RXFFIL = 2; // Issue the SPIB_RX_INT when two values are in the RX FIFO
    //    SpibRegs.SPITXBUF = 0x4A3B; // 0x4A3B and 0xB517 have no special meaning. Wanted to send
    //    SpibRegs.SPITXBUF = 0xB517; // something so you can see the pattern on the Oscilloscope

    //    // Exercise 4
    //    // Transmit SPI commands to read Gyro Z axis
    //    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // AC Clear GPIO66 Low to select MPU9250 chip
    //    SpibRegs.SPIFFRX.bit.RXFFIL = 2; // AC Issue SPIB_RX_INT when two values are in the RX FIFO
    //    SpibRegs.SPITXBUF = (0x8000 | 0x4600); // AC Read starting from GYRO_YOUT_L register (0x46)
    //    SpibRegs.SPITXBUF = 0x0000; // AC Send 16 zeros to receive the Gyro Z reading

    // Start SPI transaction to read MPU-9250 sensor data (0x3B to 0x48)
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // SS Low
    SpibRegs.SPIFFRX.bit.RXFFIL = 8;      // Interrupt when 7 values are in RX FIFO
    SpibRegs.SPITXBUF = (0x8000 | 0x3A00); // Read from 0x3A
    SpibRegs.SPITXBUF = 0x0000;           // Dummy data to read next 2 bytes
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;           // 8th transfer for 14 bytes total


    // Blink LaunchPad Red LED
    GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;

    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{

    //    //Play Happy Birthday
    //    if (CpuTimer1.InterruptCount < SONG_LENGTH) {
    //        EPwm9Regs.TBPRD = songarray[CpuTimer1.InterruptCount]; // AC change the period to period of song notes
    //    }
    //
    //    else {
    //        GPIO_SetupPinMux(16, GPIO_MUX_CPU1, 0); // Set GPIO16 as GPIO instead of PWM
    //        GPIO_SetupPinOptions(16, GPIO_OUTPUT, GPIO_PUSHPULL); // AC configure as output
    //        GpioDataRegs.GPASET.bit.GPIO16 = 1; // AC Set low
    //    }


    //    if (CpuTimer1.InterruptCount < nokiaLength) {
    //        EPwm9Regs.TBPRD = nokiaMelody[CpuTimer1.InterruptCount]; // AC change the period to period of song notes
    //    }
    //
    //    else {
    //        GPIO_SetupPinMux(16, GPIO_MUX_CPU1, 0); // Set GPIO16 as GPIO instead of PWM
    //        GPIO_SetupPinOptions(16, GPIO_OUTPUT, GPIO_PUSHPULL); // AC configure as output
    //        GpioDataRegs.GPASET.bit.GPIO16 = 1; // AC Set low
    //    }

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




__interrupt void SPIB_isr(void){

    //    //AC exercise 4
    //    spivalue1 = SpibRegs.SPIRXBUF; // AC Read first 16 bit value off RX FIFO. Probably is zero since no chip
    //    spivalue2 = SpibRegs.SPIRXBUF; // AC Read second 16 bit value off RX FIFO. Again probably zero
    //    gyroz_raw = spivalue2; // AC Extract the Gyro Z value (16-bit signed integer)
    //    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // AC Set GPIO 66 high to end Slave Select. Now to Scope. Later to deselect MPU9250.



    // AC Read 7 values from RX FIFO
    int16_t dummy = SpibRegs.SPIRXBUF;      // AC Dummy
    accel_x_raw = SpibRegs.SPIRXBUF;        // AC 0x3B-0x3C
    accel_y_raw = SpibRegs.SPIRXBUF;        // AC 0x3D-0x3E
    accel_z_raw = SpibRegs.SPIRXBUF;        // AC 0x3F-0x40
    dummy = SpibRegs.SPIRXBUF;              // AC Temperature senser
    gyro_x_raw = SpibRegs.SPIRXBUF;         // AC 0x43-0x44
    gyro_y_raw = SpibRegs.SPIRXBUF;         // AC 0x45-0x46
    gyro_z_raw = SpibRegs.SPIRXBUF;         // AC 0x47-0x48

    // AC Convert raw values
    accel_x_g = (4.0/32767.0) * accel_x_raw;
    accel_y_g = (4.0/32767.0) * accel_y_raw;
    accel_z_g = (4.0/32767.0) * accel_z_raw;
    gyro_x_dps = (250.0/32767.0) * gyro_x_raw;
    gyro_y_dps = (250.0/32767.0) * gyro_y_raw;
    gyro_z_dps = (250.0/32767.0) * gyro_z_raw;

    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // AC Set GPIO 66 high to end Slave Select. Now to Scope. Later to deselect MPU9250.


    spib_counter++; // AC Increment the counter


    if ((spib_counter % 200) == 0) { // AC print to tera term every 200ms (200 * 1ms)
        UARTPrint = 1;
    }


    // Later when actually communicating with the MPU9250 do something with the data. Now do nothing.
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Clear Overflow flag just in case of an overflow
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Clear RX FIFO Interrupt flag so next interrupt will happen
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6; // Acknowledge INT6 PIE interrupt

}

void setupSpib(void) //Call this function in main() somewhere after the DINT; line of code.
{
    int16_t temp = 0;


    // Step 1.
    // cut and paste here all the SpibRegs initializations you found for part 3.
    // Also don’t forget to cut and paste the GPIO settings for GPIO63, 64, 65, 66 which are also a part of the SPIB setup.


    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0); // Set as GPIO66 and used as MPU-9250 SS
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO66 an Output Pin
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; //Initially Set GPIO66/SS High so MPU-9250 is not selected
    GPIO_SetupPinMux(63, GPIO_MUX_CPU1, 15); //Set GPIO63 pin to SPISIMOB
    GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 15); //Set GPIO64 pin to SPISOMIB
    GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 15); //Set GPIO65 pin to SPICLKB
    EALLOW;


    GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0; // Enable Pull-ups on SPI PINs Recommended by TI for SPIPins
    GpioCtrlRegs.GPCPUD.bit.GPIO64 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3; // Set I/O pin to asynchronous mode recommended forSPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3; // Set I/O pin to asynchronous mode recommended forSPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3; // Set I/O pin to asynchronous mode recommended forSPI
    EDIS;
    // ---------------------------------------------------------------------------
    SpibRegs.SPICCR.bit.SPISWRESET = 1; // AC Put SPI in Reset (1h)
    SpibRegs.SPICTL.bit.CLK_PHASE = 1; //This happens to be the mode for both the DAN28027 and
    SpibRegs.SPICCR.bit.CLKPOLARITY = 0; //The MPU-9250, Mode 01.
    SpibRegs.SPICTL.bit.MASTER_SLAVE = 1; // Set to SPI Master (1h)
    SpibRegs.SPICCR.bit.SPICHAR = 15; // AC Set to transmit and receive 16 bits each write to SPITXBUF (Fh)
    SpibRegs.SPICTL.bit.TALK = 1; // Enable transmission
    SpibRegs.SPIPRI.bit.FREE = 1; // Free run, continue SPI operation
    SpibRegs.SPICTL.bit.SPIINTENA = 0; // Disables the SPI interrupt
    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 49 ; // AC Set SCLK to 1 MHz (50 MHz / (24 + 1)) // Set SCLK bit rate to 1 MHz so 1us period. SPI base clock is
    // 50MHZ. And this setting divides that base clock to create SCLK’s period (Double check)
    SpibRegs.SPISTS.all = 0x0000; // Clear status flags just in case they are set for some reason
    SpibRegs.SPIFFTX.bit.SPIRST = 1; // AC Pull SPI FIFO out of reset, SPI FIFO can resume transmit or receive.
    SpibRegs.SPIFFTX.bit.SPIFFENA = 1; // AC Enable SPI FIFO enhancements
    SpibRegs.SPIFFTX.bit.TXFIFO = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFTX.bit.TXFFINTCLR = 1; // Write 1 to clear SPIFFTX[TXFFINT] flag just in case it is set


    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // AC Write 1 to clear SPIFFRX[RXFFINT] flag just in case itis set
    SpibRegs.SPIFFRX.bit.RXFFIENA = 1; // AC Enable the RX FIFO Interrupt. RXFFST >= RXFFIL
    SpibRegs.SPIFFCT.bit.TXDLY = 8; // CHNAGE THIS BACKKKK  Set delay between transmits to 0 spi clocks.
    SpibRegs.SPICCR.bit.SPISWRESET = 1; // AC Pull the SPI out of reset (Double Check)
    SpibRegs.SPIFFTX.bit.TXFIFO = 1; // AC Release transmit FIFO from reset.
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1; // Re-enable receive FIFO operation
    SpibRegs.SPICTL.bit.SPIINTENA = 1; // Enables SPI interrupt. !! I don’t think this is needed. Needto Test
    SpibRegs.SPIFFRX.bit.RXFFIL =2; //Interrupt Level to 16 words or more received into FIFO causes interrupt. This is just the initial setting for the register. Will be changed below



    //-----------------------------------------------------------------------------------------------------------------


    // Step 2:
    // perform a multiple 16 bit transfer to initialize MPU-9250 registers 0x13,0x14,0x15,0x16
    // 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C 0x1D, 0x1E, 0x1F. Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low
    // Perform the number of needed writes to SPITXBUF to write to all 13 registers. Remember we are sending 16 bit transfers, so two registers at a time after the first 16 bit transfer.
    // To address 00x13 write 0x00
    SpibRegs.SPITXBUF = (0x1300 | 0x00);   // AC 0x13: 0x00 (starting address)
    // To address 00x14 write 0x00
    // To address 00x15 write 0x00
    SpibRegs.SPITXBUF = 0x0000;            // AC 0x14: 0x00, 0x15: 0x00
    // To address 00x16 write 0x00
    // To address 00x17 write 0x00
    SpibRegs.SPITXBUF = 0x0000;            // AC 0x16: 0x00, 0x17: 0x00
    // To address 00x18 write 0x00
    // To address 00x19 write 0x13
    SpibRegs.SPITXBUF = 0x0013;            // AC 0x18: 0x00, 0x19: 0x13
    // To address 00x1A write 0x02
    // To address 00x1B write 0x00
    SpibRegs.SPITXBUF = 0x0200;            // AC 0x1A: 0x02, 0x1B: 0x00
    // To address 00x1C write 0x08
    // To address 00x1D write 0x06
    SpibRegs.SPITXBUF = 0x0806;            // AC 0x1C: 0x08, 0x1D: 0x06
    // To address 00x1E write 0x00
    // To address 00x1F write 0x00
    SpibRegs.SPITXBUF = 0x0000;            // AC 0x1E: 0x00, 0x1F: 0x00
    // wait for the correct number of 16 bit values to be received into the RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST!=7); // AC Wait for 7 values 13 - 1 = 12   12/2 = 6   6 + 1 = 7
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    temp = SpibRegs.SPIRXBUF;
    // read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    temp = SpibRegs.SPIRXBUF; // Read 2
    temp = SpibRegs.SPIRXBUF; // Read 3
    temp = SpibRegs.SPIRXBUF; // Read 4
    temp = SpibRegs.SPIRXBUF; // Read 5
    temp = SpibRegs.SPIRXBUF; // Read 6
    temp = SpibRegs.SPIRXBUF; // Read 7
    DELAY_US(10); // Delay 10us to allow time for the MPU-9250 to get ready for next transfer.




    // Step 3:
    // perform a multiple 16 bit transfer to initialize MPU-9250 registers 0x23,0x24,0x25,0x26
    // 0x27, 0x28, 0x29. Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low
    // Perform the number of needed writes to SPITXBUF to write to all 7 registers
    // To address 00x23 write 0x00
    SpibRegs.SPITXBUF = (0x2300 | 0x00);   // AC 0x23: 0x00 (starting address)
    // To address 00x24 write 0x40
    // To address 00x25 write 0x8C
    SpibRegs.SPITXBUF = 0x408C;            // AC 0x24: 0x40, 0x25: 0x8C
    // To address 00x26 write 0x02
    // To address 00x27 write 0x88
    SpibRegs.SPITXBUF = 0x0288;            // AC 0x26: 0x02, 0x27: 0x88
    // To address 00x28 write 0x0C
    // To address 00x29 write 0x0A
    SpibRegs.SPITXBUF = 0x0C0A;            // AC 0x28: 0x0C, 0x29: 0x0A
    // wait for the correct number of 16 bit values to be received into the RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST!=4); // AC Wait for 4 values 7 - 1 = 6   6/2 = 3  3 + 1 = 4
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    temp = SpibRegs.SPIRXBUF;
    // read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    temp = SpibRegs.SPIRXBUF; // Read 2
    temp = SpibRegs.SPIRXBUF; // Read 3
    temp = SpibRegs.SPIRXBUF; // Read 4
    DELAY_US(10); // Delay 10us to allow time for the MPU-9250 to get ready for next transfer.


    /// perform a single 16 bit transfer to initialize MPU-9250 register 0x2A
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    // Write to address 0x2A the value 0x81
    SpibRegs.SPITXBUF = (0x2A00 | 0x0081); // AC 0x2A: 0x81
    // wait for one byte to be received
    while(SpibRegs.SPIFFRX.bit.RXFFST!=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);




    // The remainder of this code is given to you.


    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3800 | 0x0001); // 0x3800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3A00 | 0x0001); // 0x3A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6400 | 0x0001); // 0x6400
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6700 | 0x0003); // 0x6700
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6A00 | 0x0020); // 0x6A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6B00 | 0x0001); // 0x6B00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7500 | 0x0071); // 0x7500
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7700 | 0x00EB); // 0x7700
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7800 | 0x0012); // 0x7800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7A00 | 0x00E4); // 0x7A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7B00 | 0xE600); // 0x7B00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7D00 | 0x001D); // 0x7D00 //1D56
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7E00 | 0x5600); // 0x7E00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(50);

    // Clear SPIB interrupt source just in case it was issued due to any of the above initializations.
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1;  // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1;  // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}

