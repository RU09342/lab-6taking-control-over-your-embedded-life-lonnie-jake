# Lab 6: "High Power" Control
For starters, you will not be dealing with anything that is truly "high power". Instead, what I am considering "high power" is anything with the potential to damage or fry your microcontrollers if you were to drive them directly. The idea behind this part of the lab is to learn how not only to drive things that may require a high voltage or high current, but how to then protect your microcontroller from them.

## Switching
Most of you have used one of the types of switching circuits to control the RGB LEDs. For this part of the lab, you need to focus on the different types of switching circuits along with the differences in inductive and resistive loads.

### Relays
A relay is a electro-mechanical system which can open and close a switch based on an input. 
![Relay](https://www.phidgets.com/docs/images/1/1d/3051_1_Relay_Diagram.jpg)
These are extremely useful in situations where large amounts of current need to flow, such as in automotive applications, but they do have their limits. For starters, since the actuation process requires a constant current, sometimes this can be too much for your processor to handle. Second, a lot of these relays require higher than 3.3V, which limits how you can actually turn these things on and off. Using the MSP430G2553, control the state of a relay to drive a power resistor with +12V. Your README for this part should include a screenshot of the output of your MSP and the voltage across the resistor. Try to figure out the switching speed limitations of the relay experimentally.

### MOSFET Switching
The MOSFET switch is a very simple circuit which can be used in a multitude of applications. One of the most important features of the MOSFET Switch is the near zero current it takes to switch the MOSFET from an on to an off state. There are two main architectures, low-side and high-side switch, each requiring a different type of MOSFET. Using the MSP430G2553, drive a power resistor with +12V in the same fashion as the relay. Obtain an MSP430G2553 voltage output along with the voltage through the power resistor. Try to figure out the switching speed limitations of the MOSFET experimentally.

## Deliverables
Along with what was asked in each part, you will need to utilize the DMM to determine what the current draw from each switch is and if that falls into spec with the Microcontroller. You need to then come up with the best configuration you can think of using to control something that requires large current, but also protects your processor from damage. The reason I am asking you to do this with just the G2553 is: A) The code is just generating a square wave, and B) this part of the lab runs the highest chance of damaging your parts and we have spare G2553's just in case.

#Lab 6: High Power Control

##Setup
The only software componenet necessary for this part of the lab is to use the microcontroller to toggle the relay.
I used the my code from a previous lab altered slightly. All it does is throw an interrupt when the button is pressed, then flips pin 1.0.
The code is included below:

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

##Relays

The circuit we used to drive the relay with the microprocessor is below:
![RelayCircuit](https://raw.githubusercontent.com/RU09342/lab-6taking-control-over-your-embedded-life-lonnie-jake/master/High%20Power%20Control/pictures/relay%20circuit.jpg)
![relaycircuitirl]()

The configuration that was settled on was to use a NMOSFET in a low side switch configuration. The processor needed to be separated from the load of the circuit because of the high current draw of the power resistor. This configuration ensured no current would harm the processor due to MOSFETs having no input current.

We found that the power voltage across the power resistor when turned on was 10V.
The screenshots for on/off are included below.
![resistoroff](https://raw.githubusercontent.com/RU09342/lab-6taking-control-over-your-embedded-life-lonnie-jake/master/High%20Power%20Control/pictures/power%20resistor%20voltage%20off.jpg)
![resistoron](https://raw.githubusercontent.com/RU09342/lab-6taking-control-over-your-embedded-life-lonnie-jake/master/High%20Power%20Control/pictures/power%20resisotr%20voltage%20on.jpg)

###Relay Switching
The max switching speed was determined to be around 180Hz. I have included screenshots of just before the cutoff frequency and just after the cutoff frequency below.
![beforecutoff](https://raw.githubusercontent.com/RU09342/lab-6taking-control-over-your-embedded-life-lonnie-jake/master/High%20Power%20Control/pictures/relax%20at%20just%20about%20cutoff%20frequency.jpg)
![justaftercutoff](https://raw.githubusercontent.com/RU09342/lab-6taking-control-over-your-embedded-life-lonnie-jake/master/High%20Power%20Control/pictures/relay%20post%20cutoff%20frequency.jpg)

##MOSFET Switching
To compare the cutoff frequencies of the two devices, we also measured where the MOSFET cuts off. The pictures are included below.
![justbeforecutoffmos](https://raw.githubusercontent.com/RU09342/lab-6taking-control-over-your-embedded-life-lonnie-jake/master/High%20Power%20Control/pictures/mosfet%20at%20just%20about%20cutoff.jpg)
![justaftercutoffmos](https://raw.githubusercontent.com/RU09342/lab-6taking-control-over-your-embedded-life-lonnie-jake/master/High%20Power%20Control/pictures/mosfet%20after%20cutoff.jpg)
