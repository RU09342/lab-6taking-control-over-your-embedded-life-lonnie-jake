# Lab 6: "High Power" Control

## Setup
The only software componenet necessary for this part of the lab is to use the microcontroller to toggle the relay.
I used the my code from a previous lab altered slightly. All it does is throw an interrupt when the button is pressed, then flips pin 1.0.
The code is included below:

```
#include <msp430.h> 

int blahh = 0; //variable to keep track of button interrupts

void main(void)
{

WDTCTL = WDTPW + WDTHOLD; // Stop watchdog timer


P1DIR |= (BIT0); // Set LED as outputs
P1OUT &= ~(BIT0); // Initialize the LED off
P1DIR &= ~BIT3;
P1REN |= BIT3; //Enables a resistor for the button.
P1OUT |= BIT3; //Setting the resistor as a pull-up.
P1IES |= BIT3; //Sets Edge so that the button is activated when pressed down.

P1IE |= BIT3; //Sets mask so that interrupt can't be ignored

__enable_interrupt(); // enables interrupt for board

while (1) //infinite loooooooooooop
{

if(blahh > 0)
{
P1OUT |= (BIT0);
}
else P1OUT &= ~BIT0;
}

}

// Interrupt routine
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
blahh ^= 0x01;  // flips LED so that the if statement to blink the LED is activated.
P1IFG &= ~BIT3; // Clear flag when done
P1OUT &= ~BIT0; // Turn's LEDs off after flag is cleared.

}
```
## Relays

The circuit we used to drive the relay with the microprocessor is below:
![RelayCircuit](https://raw.githubusercontent.com/RU09342/lab-6taking-control-over-your-embedded-life-lonnie-jake/master/High%20Power%20Control/pictures/relay%20circuit.jpg)
![relaycircuitirl](https://raw.githubusercontent.com/RU09342/lab-6taking-control-over-your-embedded-life-lonnie-jake/master/High%20Power%20Control/pictures/relay%20circuit%20irl.jpg)

The configuration that was settled on was to use a NMOSFET in a low side switch configuration. The processor needed to be separated from the load of the circuit because of the high current draw of the power resistor. This configuration ensured no current would harm the processor due to MOSFETs having no input current.

We found that the power voltage across the power resistor when turned on was 10V.
The screenshots for on/off are included below.
![resistoroff](https://raw.githubusercontent.com/RU09342/lab-6taking-control-over-your-embedded-life-lonnie-jake/master/High%20Power%20Control/pictures/power%20resistor%20voltage%20off.jpg)
![resistoron](https://raw.githubusercontent.com/RU09342/lab-6taking-control-over-your-embedded-life-lonnie-jake/master/High%20Power%20Control/pictures/power%20resisotr%20voltage%20on.jpg)

### Relay Switching
The max switching speed was determined to be around 180Hz. I have included screenshots of just before the cutoff frequency and just after the cutoff frequency below.
![beforecutoff](https://raw.githubusercontent.com/RU09342/lab-6taking-control-over-your-embedded-life-lonnie-jake/master/High%20Power%20Control/pictures/relax%20at%20just%20about%20cutoff%20frequency.jpg)
![justaftercutoff](https://raw.githubusercontent.com/RU09342/lab-6taking-control-over-your-embedded-life-lonnie-jake/master/High%20Power%20Control/pictures/relay%20post%20cutoff%20frequency.jpg)

## MOSFET Switching
To compare the cutoff frequencies of the two devices, we also measured where the MOSFET cuts off. The pictures are included below.
![justbeforecutoffmos](https://raw.githubusercontent.com/RU09342/lab-6taking-control-over-your-embedded-life-lonnie-jake/master/High%20Power%20Control/pictures/mosfet%20at%20just%20about%20cutoff.jpg)
![justaftercutoffmos](https://raw.githubusercontent.com/RU09342/lab-6taking-control-over-your-embedded-life-lonnie-jake/master/High%20Power%20Control/pictures/mosfet%20after%20cutoff.jpg)

## Best Configuration
The best configuration for driving high current would involve a MOSFET device connecting the microprocessor to the rest of the circuit. This elminates any risk of a current getting back to the processor. For driving high currents at low-ish frequencies a relay would be preferable to a FET. These two devices in a low side switch configuration would be the best way to protect the processor from high current.
