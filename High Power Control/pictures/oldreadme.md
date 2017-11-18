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