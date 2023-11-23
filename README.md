# Temperature Control Project

## What's Inside
This is where I'm organizing the stuff for my temperature control project. It's using an STM32F411VE board, an NTC thermistor, PID controller, and a relay to control the heat.

## How to Set Up

### Hardware
1. **Connect the Thermistor:**
   - Hook up the NTC thermistor to an analog pin. Make sure it's getting power from somewhere stable and has a resistor in the mix.

2. **Relay for the Heater:**
   - Wire the relay to control the heater. Simple as that.

3. **PID Control with STM32:**
   - Figure out how to use the PID controller on the STM32F411VE. This is where the smarts come in.

### Software
1. **STM32CubeIDE Setup:**
   - Get the STM32CubeIDE set up for coding.

2. **Configure the Pins:**
   - Tell the STM32F411VE where things are connected—like which pins are for the sensors and the relay.

3. **Timer Setup:**
   - Set up a timer to keep things on schedule for the PID control.

4. **Reading Temperature:**
   - Write code to read temperature data from the NTC thermistor.

5. **Implement PID Control:**
   - Make the STM32F411VE do its PID thing. This is where the temperature magic happens.

6. **Relay Control:**
   - Make sure the relay is dancing to the PID tune.

7. **Fine-Tune the PID:**
   - Adjust the PID settings until the temperature is just right.

8. **Safety First:**
   - Add in some features to keep things safe. No one wants things getting too hot.

## Testing
1. **Simulate First:**
   - Simulate the system using STM32CubeIDE before hooking up the real hardware.

2. **Hardware Testing:**
   - Connect everything and see how it performs in the real world.

3. **Tweak as Needed:**
   - Adjust things until the temperature control is spot on.
