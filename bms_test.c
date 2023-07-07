//#############################################################################
//
// FILE:   empty_driverlib_main.c
//
//! \addtogroup driver_example_list
//! <h1>Empty Project Example</h1> 
//!
//! This example is an empty project setup for Driverlib development.
//!
//
//#############################################################################
//
//
// $Copyright:
// Copyright (C) 2023 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

//
// Included Files
//
#include "stdlib.h"
#include "driverlib.h"
#include "device.h"
#include "board.h"
//#include "vector.h"
#include "c2000ware_libraries.h"
//#include <user/LTC_SPI.h>
#include "user/LTC68041.h"

extern uint16_t RxData[2][12] = {{0,0,0,0,0,0,0,0,0,0,0,0}};
extern uint16_t AUXREG[1][6] = {{0,0,0,0,0,0}};

extern uint16_t NTC_SENSOR[2][6] = {{0,0,0,0,0,0}};

void Pulse(){
    DEVICE_DELAY_US(10);
    GPIO_writePin(OUT_CS, 1);
    DEVICE_DELAY_US(10);
    GPIO_writePin(OUT_CS, 0);
}
//
// Main
//
void main(void)
{

    //uint16_t RxData[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    //
    // Initialize device clock and peripherals
    //
    Device_init();

    //
    // Disable pin locks and enable internal pull-ups.
    //
    Device_initGPIO();

    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    //
    // PinMux and Peripheral Initialization
    //
    Board_init();
    GPIO_writePin(OUT_CS, 1);

    //
    // C2000Ware Library initialization
    //
    C2000Ware_libraries_init();

    //
    // Enable Global Interrupt (INTM) and real time interrupt (DBGM)
    //
    EINT;
    ERTM;
    bool status = false;

/*    unsigned char* msg;
    msg = "\r\n\nHello World!\0\n";
    SCI_writeCharArray(UART_BASE, (uint16_t*)msg, 17);
    msg = "\r\n\nWaiting for command...\0\n";
    SCI_writeCharArray(UART_BASE, (uint16_t*)msg, 27);*/

   /* csPulse();
    DEVICE_DELAY_US(10000);
    csPulse();


    //send adc cmd
    uint8_t cmd1[4] = {0x05, 0xef, 0xd8, 0x72};
    csPulse();
    SPI_sendNByte(SPIB_BASE, cmd1, 4 , 2);
    csPulse();

    //wait 300ms
    DEVICE_DELAY_US(300000);

    uint8_t cmd2[4] = {0x00, 0x04, 0x07, 0xc2};
    csPulse();
    SPI_sendNByte(SPIB_BASE, cmd2, 4 , 2);
    SPI_readNByte(SPIB_BASE, RxData, 8, 2);
    csPulse();*/

    wakeup_sleep();
    wakeup_sleep();
    wakeup_sleep();

    LTC6804_initialize();
    //LTC6804_adcvax();
    //LTC6804_adcv();
    //DEVICE_DELAY_US(3000);
    //LTC6804_rdcv(0,2,RxData);
    //LTC6804_rdaux(0,1, AUXREG);

    uint8_t CFGR0 = 0b11111100;
    uint8_t CFGR1 = 0x00;
    uint8_t CFGR2 = 0x00;
    uint8_t CFGR3 = 0x00;
    uint8_t CFGR4 = 0x00;
    uint8_t CFGR5 = 0b00000000;
    uint8_t cfgcmd[1][6] = {{CFGR0,CFGR1,CFGR2,CFGR3,CFGR4,CFGR5}};
    LTC6804_wrcfg(2,cfgcmd);

    uint16_t IDCOM0 = 0b0110;
    uint16_t D0 = 0b10011000;
    uint16_t FCOM0 = 0b1000;

    uint16_t IDCOM1 = 0b0000;
    uint16_t D1 = 0b00000001;
    uint16_t FCOM1 = 0b1001;




    int i;
    for(i=0;i<6;i++){

        uint8_t COMM0 = (IDCOM0 << 4) + (D0 >> 4);
        uint8_t COMM1 = (D0 << 4) + FCOM0;
        uint8_t COMM2 = (IDCOM1 << 4) + (D1 >> 4);
        uint8_t COMM3 = (D1 << 4) + FCOM1;
        uint8_t COMM4 = 0x00;
        uint8_t COMM5 = 0x00;
        uint8_t commcmd[1][6] = {{COMM0,COMM1,COMM2,COMM3,COMM4, COMM5}};

        LTC6804_wrcomm(1,commcmd);

        LTC6804_stcomm();


        DEVICE_DELAY_US(500);
        LTC6804_adax();
        DEVICE_DELAY_US(230);
        LTC6804_rdaux(0,1, AUXREG);

        NTC_SENSOR[0][i] = AUXREG[0][0];
        D1 = D1 << 1;
    }











    while(1){
        GPIO_writePin(AMS_STATUS, 1);
        GPIO_writePin(DEVICE_GPIO_PIN_LED1, status);
        status = !status;
        DEVICE_DELAY_US(500000);
        wakeup_sleep();
        //wakeup_idle();
        //LTC6804_wrcfg(1,cfgcmd);

    }
}

//
// End of File
//
