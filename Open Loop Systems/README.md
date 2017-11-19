# Lab 6: Open Loop Systems
## Board Choice
We chose to use the 6989 for this part of the lab because we can do all PWM and ADC while reading out the voltages of the ADC to the LCD while we're doing the lab. 

The code used for this lab is below:
```
#include <msp430.h>
#include "LCDDriver.h"

void uart_init();
void timer_init();
void LCDInit();
void PrepGPIO();
int tempToPWM(int temp);

int temp;
int pwm;

const int convertToASCII = 48; //Add this to a number to convert to ASCII character

int adcReading;
int d_four, d_three, d_two, d_one;

int main(void)
{
  WDTCTL = WDTPW | WDTHOLD;                 // Stop Watchdog

  // Disable the GPIO power-on default high-impedance mode to activate
  // previously configured port settings
  PM5CTL0 &= ~LOCKLPM5;

  LCDInit();

  //Configure LED
  PrepGPIO();
  P1DIR |= BIT0;
  P9DIR |= BIT7;

  uart_init();
  timer_init();

  //Configure ADC input
  P1DIR |= BIT3;
  P1SEL0 |= BIT3;
  P1SEL1 |= BIT3;

  // Configure ADC12
  ADC12CTL0 = ADC12SHT0_2 | ADC12ON;      // Sampling time, S&H=16, ADC12 on
  ADC12CTL1 = ADC12SHP;                   // Use sampling timer
  ADC12CTL2 |= ADC12RES_2;                // 12-bit conversion results
  ADC12MCTL0 |= ADC12INCH_3;              // A3 ADC input select; Vref=AVCC
  ADC12IER0 |= ADC12IE0;                  // Enable ADC conv complete interrupt

  while (1)
     {
      __delay_cycles(1000);
      ADC12CTL0 |= ADC12ENC | ADC12SC;    // Start sampling/conversion
      __bis_SR_register(LPM0_bits | GIE); // LPM0, ADC12_ISR will force exit
      __no_operation();                   // For debugger
     }
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A0_VECTOR))) USCI_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
  switch(__even_in_range(UCA0IV, USCI_UART_UCTXCPTIFG))
  {
    case USCI_NONE: break;
    case USCI_UART_UCRXIFG:
      while(!(UCA0IFG&UCTXIFG));
      temp = UCA0RXBUF;
      pwm = tempToPWM(temp);
      TB0CCR2 = 255 - pwm;
      UCA0TXBUF = UCA0RXBUF;
      __no_operation();
      break;
    case USCI_UART_UCTXIFG: break;
    case USCI_UART_UCSTTIFG: break;
    case USCI_UART_UCTXCPTIFG: break;
  }
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(ADC12_VECTOR))) ADC12_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(ADC12IV, ADC12IV_ADC12RDYIFG))
    {
        case ADC12IV_NONE:        break;    // Vector  0:  No interrupt
        case ADC12IV_ADC12OVIFG:  break;    // Vector  2:  ADC12MEMx Overflow
        case ADC12IV_ADC12TOVIFG: break;    // Vector  4:  Conversion time overflow
        case ADC12IV_ADC12HIIFG:  break;    // Vector  6:  ADC12BHI
        case ADC12IV_ADC12LOIFG:  break;    // Vector  8:  ADC12BLO
        case ADC12IV_ADC12INIFG:  break;    // Vector 10:  ADC12BIN
        case ADC12IV_ADC12IFG0:             // Vector 12:  ADC12MEM0 Interrupt
            adcReading = ADC12MEM0;
            d_four = (int)(adcReading/1000);
            adcReading = adcReading % 1000;
            d_three = (int)(adcReading/100);
            adcReading = adcReading % 100;
            d_two = (int)(adcReading/10);
            d_one = adcReading % 10;
            showChar(d_four + convertToASCII, 3);
            showChar(d_three + convertToASCII, 4);
            showChar(d_two + convertToASCII, 5);
            showChar(d_one + convertToASCII, 6);
            if (ADC12MEM0 >= 0x7ff)         // ADC12MEM0 = A1 > 0.5AVcc?
                P1OUT |= BIT0;              // P1.0 = 1
            else
                P1OUT &= ~BIT0;             // P1.0 = 0
            __delay_cycles(1600000);
                // Exit from LPM0 and continue executing main
                __bic_SR_register_on_exit(LPM0_bits);
            break;
        default: break;
    }
}

void uart_init()
{
    // Configure GPIO
    P2SEL0 |= BIT0 | BIT1;                    // USCI_A0 UART operation
    P2SEL1 &= ~(BIT0 | BIT1);

    // Startup clock system with max DCO setting ~8MHz
    CSCTL0_H = CSKEY >> 8;                    // Unlock clock registers
    CSCTL1 = DCOFSEL_3 | DCORSEL;             // Set DCO to 8MHz
    CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;     // Set all dividers
    CSCTL0_H = 0;                             // Lock CS registers

    // Configure USCI_A0 for UART mode
    UCA0CTLW0 = UCSWRST;                      // Put eUSCI in reset
    UCA0CTLW0 |= UCSSEL__SMCLK;               // CLK = SMCLK
    // Baud Rate calculation
    // 8000000/(16*9600) = 52.083
    // Fractional portion = 0.083
    // User's Guide Table 21-4: UCBRSx = 0x04
    // UCBRFx = int ( (52.083-52)*16) = 1
    UCA0BR0 = 52;                             // 8000000/16/9600
    UCA0BR1 = 0x00;
    UCA0MCTLW |= UCOS16 | UCBRF_1 | 0x4900;
    UCA0CTLW0 &= ~UCSWRST;                    // Initialize eUSCI
    UCA0IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt
}

void PrepGPIO()
{
    P1DIR = 0xFF; P2DIR = 0xFF;
    P1REN = 0xFF; P2REN = 0xFF;
    P1OUT = 0x00; P2OUT = 0x00;
}


void LCDInit()
{
    PJSEL0 = BIT4 | BIT5;                   // For LFXT

    // Initialize LCD segments 0 - 21; 26 - 43
    LCDCPCTL0 = 0xFFFF;
    LCDCPCTL1 = 0xFC3F;
    LCDCPCTL2 = 0x0FFF;

    // Disable the GPIO power-on default high-impedance mode
    // to activate previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;

    // Configure LFXT 32kHz crystal
    CSCTL0_H = CSKEY >> 8;                  // Unlock CS registers
    CSCTL4 &= ~LFXTOFF;                     // Enable LFXT
    do
    {
      CSCTL5 &= ~LFXTOFFG;                  // Clear LFXT fault flag
      SFRIFG1 &= ~OFIFG;
    }while (SFRIFG1 & OFIFG);               // Test oscillator fault flag
    CSCTL0_H = 0;                           // Lock CS registers

    // Initialize LCD_C
    // ACLK, Divider = 1, Pre-divider = 16; 4-pin MUX
    LCDCCTL0 = LCDDIV__1 | LCDPRE__16 | LCD4MUX | LCDLP;

    // VLCD generated internally,
    // V2-V4 generated internally, v5 to ground
    // Set VLCD voltage to 2.60v
    // Enable charge pump and select internal reference for it
    LCDCVCTL = VLCD_1 | VLCDREF_0 | LCDCPEN;

    LCDCCPCTL = LCDCPCLKSYNC;               // Clock synchronization enabled

    LCDCMEMCTL = LCDCLRM;                   // Clear LCD memory
    //Turn LCD on
    LCDCCTL0 |= LCDON;
}

void timer_init()
{
    // Configure GPIO
    P3DIR |= BIT6;                     // P3.5 and P3.6 output
    P3SEL1 |= BIT6;                    // P3.5 and P3.6 options select

    TB0CCR0 = 255;                         // PWM Period
    TB0CCTL2 = OUTMOD_3;                      // CCR2 reset/set
    TB0CCR2 = 128;                            // CCR2 PWM duty cycle
    TB0CTL = TBSSEL__SMCLK | ID_2 | MC_1;  // SMCLK, up mode, clear TBR
}

int tempToPWM(temp)
{
    int pwm;
    pwm = -3.37*(temp-57.38);
    pwm = pwm*2.55;
    return pwm;
}
```
UART implemented by William Goh from TI.

This code takes in a PWM over UART and sets the PWM pin to a set duty cycle. This duty cycle relates to a fan speed that was determined by our system modeling below.

The circuit that was used for this open loop.
![circuit](https://raw.githubusercontent.com/RU09342/lab-6taking-control-over-your-embedded-life-lonnie-jake/master/Open%20Loop%20Systems/pictures/circuit%20diagram.PNG)
![circuitirl](https://raw.githubusercontent.com/RU09342/lab-6taking-control-over-your-embedded-life-lonnie-jake/master/Open%20Loop%20Systems/pictures/20171118_205649.jpg)

#System Modeling
Lonnie iterated through PWM cycles 10% at a time, (0%, 10%, 20%, 30%...) and measured the steady state ADC reading and actual temperature of the regulator package using a thermocouple. Once everything was tested he graphed the data in excel to get us an equation for each variable. 
![table](https://raw.githubusercontent.com/RU09342/lab-6taking-control-over-your-embedded-life-lonnie-jake/master/Open%20Loop%20Systems/pictures/table.PNG)
![tempvspwm](https://raw.githubusercontent.com/RU09342/lab-6taking-control-over-your-embedded-life-lonnie-jake/master/Open%20Loop%20Systems/pictures/temperaturevspwm.PNG)
![adcvstemp](https://raw.githubusercontent.com/RU09342/lab-6taking-control-over-your-embedded-life-lonnie-jake/master/Open%20Loop%20Systems/pictures/adcvstemperature.PNG)

We used the equation given by these two graphs to make a function called "tempToPWM" to take in a target temperature over UART, and set the correct fan speed based on the trendline.

The graphs show the expected range of values. At 100% fan speed you're only going to reach ~30C. At 0% fan speed the regulator sits at a warm ~60C.

Believe it or not, up to this point, any time that you have wanted to control your LED color or brightness so far, you have been attempting to control an Open Loop System. Basically, when in your code you state that you want a certain brightness or even a duty cycle, you are going on blind faith that the output is actually what it is supposed to be. If something seemed off, you probably went back into the code and tweaked some values. In the case of actual Systems and Control Theory, you are the feedback loop, providing some corrective signal to the system to help obtain a closer output, and we will deal with this in the Milestone. For now, we need to focus on system modeling getting a system to a desirable state. For this lab, you will be attempting to keep a voltage regulator within a specific temperature range using a DC fan which you will have control over. For this part to be a success, you need to figure out what is the minimum fan speed you need to cool off the regulator so that is stays operational.

## Voltage Regulator
You will need to take a 5V regulator from the back of the lab and drop the output across a 100 ohm power resistor (1 watt should do the trick). The input to the voltage regulator will be between 15-20V. I am giving you a range because when it is dropping a ton of voltage, it may not be able to cool it off enough with just a fan. Most of the voltage regulators in the back will have a tab on the top which we can place a thermistor to. If provided, you can use that tab, or place a through-hole thermistor making contact to the component on your board.

## Fan Control
It will be up to you, the engineer, to decide which method you want to use to control the DC fan. Most of these fans run off of 5V, and as such can not directly be driven by your microcontroller. Depending on the type of fan you use, some can take in a PWM control signal, others will need to have the voltage be modified. Since we are not providing you with any mechanical mounts, you are free to place the fan in whatever orientation you wish, so long as it is safe to operate.

## Temperature Reading
It would be really useful to see what the temperature of your system is so you can determine the performance of your system. This can be done either by displaying the current temperature over a display, passing the information over UART, or other ways as well. Remember that UART is Asynchronous, meaning that you can send information whenever you would like from your controller back to a PC, it doesn't have to be reactionary. If you used MATLAB in Lab 5, you could even plot the temperature over time which could be extremely useful in figuring out whether your system is actually doing something. 


## System Modeling
For starters, you need to figure out with your fan at MAX what the temperature that the voltage regulator reaches. Your thermistors/PTATs/Whatever else you want to use to measure temperature may not be calibrated, so your results may be consistently off with reality, but for now that is ok. After figuring this out, in increments of about 5C, see what fan speed you need to have to maintain that temperature. Do this until your regulator gets down to about 30C-40C, keeping a record of what your applied Duty Cycles or voltages were. Then using this information, attempt to find a transfer function between the applied input and the resulting temperature to model the system behavior. A simple way to do this is in MATLAB or Excel to plot your applied input on the x-axis, and your steady state temperature on your y-axis and attempt a line fit.

## Open Loop Control System
You then need to use this information to make a final open loop control system where a user can state what temperature they want the regulator to be maintained at, and the microcontroller will basically calculate/look up what it needs to set the fan to. Do not over complicate this and make it some elaborate system. All this needs to do is some math and set a duty cycle or voltage, and display in some capacity the current temperature of the system as you are measuring it.


## Deliverables
Your README needs to contain schematics of your system, the plot of the plot of the temperature and input voltages at the 5C steps, and a brief talk about why you chose the processor you did along with the control technique for the fan. As always, you should include a brief description of the code you generated to run the experiment. You need to also include information on how to use your control software, including what inputs it is expecting and what range of values you are expecting. At this time you are not going to need to user-proof the system, but you will for the milestone, so keep it in the back of your head.

### What your README and code doesn't need
For starters, note the fact I ask you to do this with only one board. You also do not need to give me all of your code in the README, just tell me functionally what is going on along with showing off any functions you may have made.

Your code *DOES NOT* need to perform any sort of closed loop control. Save that for the milestone. This means that your system does not need to try to actively change the fan speed without your help. You are going to essentially make your microcontroller one big Convert-o-Box to turn a desired temperature into a controllable signal, and then be able to read a temperature.
