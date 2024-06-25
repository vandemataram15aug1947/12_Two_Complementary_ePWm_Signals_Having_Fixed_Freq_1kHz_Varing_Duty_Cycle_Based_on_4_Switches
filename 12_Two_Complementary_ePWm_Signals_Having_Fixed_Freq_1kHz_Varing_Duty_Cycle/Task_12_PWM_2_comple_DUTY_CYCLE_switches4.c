
/////////////////////////////// Task-12  ////////////////////////////////////////////////////////////

Generate two PWM signals with a fixed  frequency of 1KHz by varying  duty cycle , such that when
one PWM signal having a duty Cycle of 95% then other PWM signal should have a
Duty cycle of 5%
The two PWM signal duty cycle must vary between 0 and 100%.
1. When switch 1 is pressed then duty cycle should increase by 10%
2. When switch 2 is pressed then duty cycle should increase by 1%
3. When switch 3 is pressed then duty cycle should decrease by 10%
3. When switch 4 is pressed then duty cycle should decrease by 1%
Two PWM Signals should be complementary
 */
#include "F28x_Project.h"
#include "F2837xD_device.h"
#include "F2837xD_Examples.h"
#include "driverlib.h"
#include "device.h"


// Function Prototypes

void initEPWM1(void);
void initEPWM2(void);
void Timer_Delay(void);
void PinMux_init();
__interrupt void xINT1_isr(void);
__interrupt void xINT2_isr(void);
__interrupt void xINT3_isr(void);
__interrupt void xINT4_isr(void);
float Duty_Cycle= 0.0;
float frequency_inc = 0.0;
int i;

void main(void)
{


    // Initialize device clock and peripherals

    Device_init();


    // Disable pin locks and enable internal pull-ups.

    Device_initGPIO();


    // Initialize PIE and clear PIE registers. Disables CPU interrupts.

    Interrupt_initModule();


    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).

    Interrupt_initVectorTable();


    //////////////  Vande  ///////////////////

    // This is needed to write to EALLOW protected registers
    EALLOW;
    // Configure XINT1 in pie Vector Table
    PieVectTable.XINT1_INT = &xINT1_isr;
    // Configure XINT2 in pie Vector Table
    PieVectTable.XINT2_INT = &xINT2_isr;
    // Configure XINT3 in pie Vector Table
    PieVectTable.XINT3_INT = &xINT3_isr;
    // Configure XINT4 in pie Vector Table
    PieVectTable.XINT4_INT = &xINT4_isr;
    EDIS;




    // Enable CPU INT1 which is connected to CPU-Timer 0
    IER |= M_INT1; // M- means to Enable
    IER |= M_INT12; // M- means to Enable

    //////////////  Vande  ///////////////////

    // Enable  ,XINT1,XINT2,XINT3 ,XINT4    in the PIE: Group 1 & 12 __interrupt 4,5,1,2

    // Enable the PIE block
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;
    // Enable PIE Group 1 INT4
    PieCtrlRegs.PIEIER1.bit.INTx4 = 1;
    // Enable PIE Group 1 INT5
    PieCtrlRegs.PIEIER1.bit.INTx5 = 1;
    // Enable PIE Group 12 INT1
    PieCtrlRegs.PIEIER12.bit.INTx1 = 1;
    // Enable PIE Group 12 INT2
    PieCtrlRegs.PIEIER12.bit.INTx2 = 1;

    //////////////  Vande  ///////////////////

    //Configure GPIO67,GPIO22,GPIO11,GPIO14 FOR xINT1,xINT2xINT3,xINT4
    GPIO_SetupXINT1Gpio(67);
    GPIO_SetupXINT2Gpio(22);
    GPIO_SetupXINT3Gpio(11);
    GPIO_SetupXINT4Gpio(14);

    //////////////  Vande  ///////////////////

    // Check switch state for falling edge
    XintRegs.XINT1CR.bit.POLARITY = 1;          // Rising edge interrupt
    XintRegs.XINT2CR.bit.POLARITY = 1;          // Rising edge interrupt
    XintRegs.XINT3CR.bit.POLARITY = 1;          // Rising edge interrupt
    XintRegs.XINT4CR.bit.POLARITY = 1;          // Rising edge interrupt

    // Calling Function of Pin Mux
    PinMux_init();


    // Initialize ePWM1 and ePWM2
    //
    initEPWM1();
    initEPWM2();

    //////////////  Vande  ///////////////////

    //Disable xINT1,xINT2,xINT3,xINT4
    XintRegs.XINT1CR.bit.ENABLE = 0;            // Disable XINT1
    XintRegs.XINT2CR.bit.ENABLE = 0;            // Disable XINT2
    XintRegs.XINT3CR.bit.ENABLE = 0;            // Disable XINT3
    XintRegs.XINT4CR.bit.ENABLE = 0;            // Disable XINT4

    // Enable Global Interrupt and Real time Interrupt
    EINT;
    ERTM;



    while(1){

        //////////////  Vande  ///////////////////

        // Enable XINT1
        XintRegs.XINT1CR.bit.ENABLE = 1;
        // Enable XINT2
        XintRegs.XINT2CR.bit.ENABLE = 1;
        // Enable XINT1
        XintRegs.XINT3CR.bit.ENABLE = 1;
        // Enable XINT2
        XintRegs.XINT4CR.bit.ENABLE = 1;
        // Wait for CPU to wake up when Interrupt is Trigger
        asm("IDLE:");
    }
}


// initEPWM1 - Configure ePWM1

void initEPWM1()
{
    //////////////  Vande  ///////////////////

    // Set frequency 30KHz
        EPwm1Regs.TBPRD = 1666.66;
        // Phase is 0
        EPwm1Regs.TBPHS.bit.TBPHS = 0x0000;
        EPwm1Regs.TBCTR = 0x0000;
        // Set compare A value  Duty cycle of 5%
        EPwm1Regs.CMPA.bit.CMPA = 1583.32;

        //////////////  END ///////////////////

      // Setup counter mode  : Count up and down
       EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
       // Clock ratio to SYSCLKOUT : High speed pre-scalar clock division
       EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
       EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

       //////////////  Vande  ///////////////////

       ///////////     These Two Register shows Complementary PWM     ////////////
       EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;    // Compare UP
       EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;   // Compare Down
}
void initEPWM2()
{
    //////////////  Vande  ///////////////////

    // Set frequency 30KHz
        EPwm2Regs.TBPRD = 1666.66;
        // Phase is 0
        EPwm2Regs.TBPHS.bit.TBPHS = 0x0000;
        EPwm2Regs.TBCTR = 0x0000;
        // Set compare A value  Duty cycle of 95%
        EPwm2Regs.CMPA.bit.CMPA = 83.33;

        //////////////  END  ///////////////////

      // Setup counter mode  :  Count up and down
       EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; //
       // Clock ratio to SYSCLKOUT   : High speed pre-scalar clock division
       EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
       EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;

       //////////////  Vande  ///////////////////

       ///////////     These Two Register shows Complementary PWM     ////////////
       EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;  // Compare UP
       EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;    // Compare Down
}


void PinMux_init()
{
    //////////////  Vande  ///////////////////
    EALLOW;
    //EPWM1 -> myEPWM1 Pinmux
    // gpio 15 to gpio 0  general purpouse I/O
    GpioCtrlRegs.GPAMUX1.all=0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO0=1;// epwm active
    GpioCtrlRegs.GPAMUX1.bit.GPIO2=1;
    EDIS;

}

//////////////  Vande  ///////////////////

__interrupt void xINT1_isr(void)
{
    // Disable XINT1
    XintRegs.XINT1CR.bit.ENABLE = 0;
        int debounce_count = 10;
            while(debounce_count > 0)
            {
                // Wait for a short delay
                DELAY_US(50000);
                // Check the state of GPIO 67
                if(GpioDataRegs.GPCDAT.bit.GPIO67 == 1)
                {
                // Ignore the edge if the input is still high
                 debounce_count = 10;
                }
                else
                {
                 // Decrement the debounce counter
                debounce_count--;
                }
            }
// Increase DutyCycle by 10%
      Duty_Cycle +=10;

      if(Duty_Cycle >=95)
      {
          Duty_Cycle=95;
      }
      // Set compare A value to update duty cycle of EPwm1
       EPwm1Regs.CMPA.bit.CMPA = (1666-(16.6*Duty_Cycle));
       // Set compare A value to update duty cycle of EPwm2
       EPwm2Regs.CMPA.bit.CMPA = (1666-(16.6*Duty_Cycle));
       // clear interrupt flag
      PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//////////////  Vande  ///////////////////

__interrupt void xINT2_isr(void)
{
    // Disable XINT2
    XintRegs.XINT2CR.bit.ENABLE = 0;
        int debounce_count = 10;
            while(debounce_count > 0)
            {
                // Wait for a short delay
                DELAY_US(50000);
                // Check the state of GPIO 22
                if(GpioDataRegs.GPADAT.bit.GPIO22 == 1)
                {
                // Ignore the edge if the input is still high
                 debounce_count = 10;
                }
                else
                {
                 // Decrement the debounce counter
                debounce_count--;
                }
            }
            // Increase DutyCycle by 1%
    Duty_Cycle +=1;

         if(Duty_Cycle >=95)
         {
             Duty_Cycle=95;
         }
         // Set compare A value to update duty cycle of EPwm1
             EPwm1Regs.CMPA.bit.CMPA = (1666-(16.6*Duty_Cycle));
             // Set compare A value to update duty cycle of EPwm1
             EPwm2Regs.CMPA.bit.CMPA = (1666-(16.6*Duty_Cycle));
             // clear interrupt flag
      PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;


}

//////////////  Vande  ///////////////////

__interrupt void xINT3_isr(void)
{
    // Disable XINT3
    XintRegs.XINT3CR.bit.ENABLE = 0;
        int debounce_count = 10;
            while(debounce_count > 0)
            {
                // Wait for a short delay
                DELAY_US(50000);
                // Check the state of GPIO 11
                if(GpioDataRegs.GPADAT.bit.GPIO11 == 1)
                {
                // Ignore the edge if the input is still high
                 debounce_count = 10;
                }
                else
                {
                 // Decrement the debounce counter
                debounce_count--;
                }
            }
            // Decrease DutyCycle by 10%
    Duty_Cycle -=10;

         if(Duty_Cycle <=5)
         {
             Duty_Cycle=5;
         }
         // Set compare A value to update duty cycle of EPwm1
             EPwm1Regs.CMPA.bit.CMPA = (1666-(16.6*Duty_Cycle));
             // Set compare A value to update duty cycle of EPwm2
             EPwm2Regs.CMPA.bit.CMPA = (1666-(16.6*Duty_Cycle));
             // clear interrupt flag
      PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;


}

//////////////  Vande  ///////////////////

__interrupt void xINT4_isr(void)
{
    // Disable XINT4
    XintRegs.XINT4CR.bit.ENABLE = 0;
        int debounce_count = 10;
            while(debounce_count > 0)
            {
                // Wait for a short delay
                DELAY_US(50000);
                // Check the state of GPIO 14
                if(GpioDataRegs.GPADAT.bit.GPIO14 == 1)
                {
                // Ignore the edge if the input is still high
                 debounce_count = 10;
                }
                else
                {
                 // Decrement the debounce counter
                debounce_count--;
                }
            }
            // Decrease DutyCycle by 1%
    Duty_Cycle -=1;

         if(Duty_Cycle <=5)
         {
             Duty_Cycle=5;
         }
          // Set compare A value to update duty cycle of EPwm1
             EPwm1Regs.CMPA.bit.CMPA = (1666-(16.6*Duty_Cycle));
             // Set compare A value to update duty cycle of EPwm2
             EPwm2Regs.CMPA.bit.CMPA = (1666-(16.6*Duty_Cycle));
             // clear interrupt flag
      PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;


}

