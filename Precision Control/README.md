# Lab 6: Precision Control
Some applications require large amounts of voltage or current, so switching techniques must be used in order to provide the desired output. Other cases however require a finer control over the voltage or current going into them (some even require a control over resistance). So far you have looked at PWM to control the brightness of an LED, is there a way to use this to output a specified voltage or current, or even a specific waveform?

## PWM Part 2
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

The circuit we used for the low pass is below:
[!lowpass](https://raw.githubusercontent.com/RU09342/lab-6taking-control-over-your-embedded-life-lonnie-jake/master/Precision%20Control/pictures/lowpass.PNG)
Our IRL performance was:
[!lowpassirl](https://raw.githubusercontent.com/RU09342/lab-6taking-control-over-your-embedded-life-lonnie-jake/master/Precision%20Control/pictures/scope_21.png)
The performance was as expected. The low pass smoothed out the transition to the top. It measured about 2.3V in real life. 


## R2R DAC
The code used to generate the sweep that we're looking for is included below:
```
int main(void)
{
  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
  //uart_init();
  //timer_init();
  P6DIR |= BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5 + BIT6;
  P3OUT |= BIT4;
  while(1)
  {
      for(i=0;i < 128; i++)
      {
          P6OUT = i;
      }
      P3OUT ^= BIT4;
      P3OUT ^= BIT4;
      for(i=127; i>=0;i--){
          P6OUT = i;
      }
  }

  __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0, interrupts enabled
  __no_operation();                         // For debugger
}
```
Pin6.0 is the least significant bit here and it goes up to P6.6, then we had to put the most significant bit on P3.4 because we ran out of GPIO pins. So the first for loop and the first XOR make the ramp up, the second XOR and the second for loop make the ramp down. The ramp going down is the full triangle wave. The circuit that we used is the diagram below...
[!r2rcircuit](https://raw.githubusercontent.com/RU09342/lab-6taking-control-over-your-embedded-life-lonnie-jake/master/Precision%20Control/R2R_Ladder.JPG)
The ramp up screenshot is below:
[!r2rup](https://raw.githubusercontent.com/RU09342/lab-6taking-control-over-your-embedded-life-lonnie-jake/master/Precision%20Control/pictures/R2R%20DAC.png)
The full ramp screenshot is below:
![!r2rramp](https://raw.githubusercontent.com/RU09342/lab-6taking-control-over-your-embedded-life-lonnie-jake/master/Precision%20Control/pictures/r2rfft/bit7.png)

## Loading Effects
For loading effect I chose 4 resistors: 100, 1000, 10k, 1M ohms. The screenshots for each one can be found [here](https://github.com/RU09342/lab-6taking-control-over-your-embedded-life-lonnie-jake/tree/master/Precision%20Control/pictures/loads).
The order is ascending, so scope 23 is for the 100 ohm and scope 26 is for the 1M ohm respectively. 

As we increased the load, the output voltage increased.

## Deliverables
The fft screenshots at the output of each bit are found [here](https://github.com/RU09342/lab-6taking-control-over-your-embedded-life-lonnie-jake/tree/master/Precision%20Control/pictures/r2rfft). Each picture is labeled for what bit it corresponds to. As I said before the triangle wave was made by just descending from the ramp we made in the above implementation.

## Bill of Materials
http://www.digikey.com/short/q34jrm
The ordering quantity reflect the minimum required to order.
