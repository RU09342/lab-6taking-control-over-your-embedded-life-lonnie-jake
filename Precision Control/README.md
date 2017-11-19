# Lab 6: Precision Control
Some applications require large amounts of voltage or current, so switching techniques must be used in order to provide the desired output. Other cases however require a finer control over the voltage or current going into them (some even require a control over resistance). So far you have looked at PWM to control the brightness of an LED, is there a way to use this to output a specified voltage or current, or even a specific waveform?

## PWM Part 2
Since you already have the code to perform PWM, then really, the software side of this part of the lab is fairly easy. You need to design a system which can take in a PWM duty cycle over something like UART (or you could have your system read in the position of a potentiometer), and produce that signal on a GPIO. The interesting part comes in when I want the output of your system to be an Analog voltage. In this case, a PWM with a 50% duty cycle should produce roughly Vcc/2 volts. This part of the lab should be done with the MSP430F5529 and the physical circuit should be constructed of an active Low-Pass Filter.

The code to take a PWM input over GPIO is included below:

```
#include <msp430.h>


void uart_init();
void timer_init();

int main(void)
{
  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
  uart_init();
  timer_init();

  __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0, interrupts enabled
  __no_operation();                         // For debugger
}

// Echo back RXed character, confirm TX buffer is ready first
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
  switch(__even_in_range(UCA1IV,4))
  {
  case 0:break;                             // Vector 0 - no interrupt
  case 2:                                   // Vector 2 - RXIFG
    while (!(UCA1IFG&UCTXIFG));             // USCI_A1 TX buffer ready?
    TA0CCR1 = 255-UCA1RXBUF;                // Set CCR1 to new PWM using input from UART
    UCA1TXBUF = UCA1RXBUF;                  // TX -> RXed character
    break;
  case 4:break;                             // Vector 4 - TXIFG
  default: break;
  }
}

void uart_init()
{
    WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
    P4SEL = BIT5+BIT4;                        // P4.4,5 = USCI_A1 TXD/RXD
    UCA1CTL1 |= UCSWRST;                      // **Put state machine in reset**
    UCA1CTL1 |= UCSSEL_2;                     // SMCLK
    UCA1BR0 = 6;                              // 1MHz 9600 (see User's Guide)
    UCA1BR1 = 0;                              // 1MHz 9600
    UCA1MCTL = UCBRS_0 + UCBRF_13 + UCOS16;   // Modln UCBRSx=0, UCBRFx=0,
                                                // over sampling
    UCA1CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
    UCA1IE |= UCRXIE;                         // Enable USCI_A1 RX interrupt
}
void timer_init()
{
        P1SEL |= BIT2;
        P1DIR |= BIT2;
        P1OUT &= ~BIT2;                       //GPIO setup for Hardware PWM pin

        TA0CTL |= TASSEL_2 |ID_2 | MC_1;      //SMCLK, Divide by 4, up mode

        TA0CCR0 = 255;              // Set clock period

        TA0CCR1 = 125;              //Defaul ~50% duty
        TA0CCTL1 |= OUTMOD_3;       // Set/Reset mode
}
```
The code for UART setup is taken from the TI resource explorer, Bhargavi Nisarga.

The circuit we used is


## R2R DAC
What if your system is noise sensitive or possibly needs more precision than just a PWM signal, you might need to look into using an actual Digital-to-Analog converter. One of the simplest DAC architectures is a R2R ladder. Using the MSP430F5529, you need to generate an 8-bit R2R ladder circuit that can produce "255" voltages between 0V and Vcc. Now how are you actually going to test this, cause I am sure you aren't going to measure 255 voltages on the DMM. You should set up your F5529 so it generates a staircase using a binary counter and then record on the oscilloscope the resulting waveform.

## Loading Effects
Obviously you are going to be making this type of circuitry to drive something. This introduces the idea of loading effect, wherein your circuit will perform differently based on what is going to be attached to it. For each of these implementations, try placing a variety of resistors from 100 ohms up to see what happens to your output signal and comment on what is happening.

## Deliverables
Along with what was asked in each of the parts above, for each implementation, you need to generate at least one triangle wave from your microntroller. This can be done by simply incrementing and decrementing values that are being sent to your circuit. You need to measure the output of each one of these along with taking the FFT on the scope of each one. The span on the FFT will need to go from 1kHz to about 50kHz if possible. You then need to compare the integrity of each signal by analyzing the difference in frequency components.

The README for this part is going to be mainly about the results of your measurement along with information on the implementation. You need to also talk about how you generated the triangle wave, but do not give me a dissertation on it. Since this is going to be talking about hardware, you need to place in the README a Bill Of Materials listing all hardware used as well as link to a Digikey cart which contains the parts needed in the right quantity. You do not need to include things like your F5529 or the breadboard or wires.
