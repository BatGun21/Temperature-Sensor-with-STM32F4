# Temperature Controller with PID

## Project Overview

This project involves the creation of a temperature controller using a Proportional-Integral-Derivative (PID) control loop. The controller regulates the temperature inside a medium-sized box by turning a 60W bulb on and off. The implementation includes calibration of the temperature sensor, polynomial interpolation, and fine-tuning of PID parameters.

## Hardware Configuration

- **Temperature Sensor:** Located inside the box to measure the ambient temperature. The sensor is interfaced using a voltage divider to adapt the output for the Analog-to-Digital Converter (ADC).
- **Actuator (Bulb):** A 60W bulb controlled by the PID algorithm to adjust the temperature. Relay control is implemented with a pull-up resistor.
- **Microcontroller:** STM32F4 is used to implement the PID control loop.

## Calibration of Temperature Sensor

The temperature sensor is calibrated using a interpolation (see inside the Caliberation folder). The calibration involves collecting data at various known temperatures. The data is then processed, and a 4th-degree polynomial equation is derived through interpolation. This equation maps sensor readings to actual temperatures, enabling accurate temperature measurement.

## PID Control Loop

The PID control loop is implemented in the STM32F4 microcontroller. The three components of the PID controller are tuned as follows:

1. **Proportional (P) Tuning:**
   - Start with P=0 and gradually increase it until the system shows signs of oscillation.
   - Identify the P value that causes oscillation and divide it by 2. This becomes the starting P value.

2. **Integral (I) Tuning:**
   - Begin with I=0 and increase it incrementally.
   - Adjust the set point to observe the controller's reaction.
   - The goal is to achieve an optimal response time without causing oscillations.
   - If oscillations occur, decrease the I value until stability is regained.

3. **Derivative (D) Tuning:**
   - Set D=0

## Usage

1. **Setup:**
   - Connect the STM32F4 microcontroller to the hardware setup.
   - Ensure the temperature sensor is properly placed inside the box in shadow (so that it does not radiation heat)

2. **Compile and Upload:**
   - Upload the compiled code to the microcontroller.

3. **Run:**
   - Power on the system and observe the temperature control in action.
   - Monitor the temperature readings through the UART interface and then plot using the Temperature Plot (see folder for Plotting).

## Notes

- **Noise Filtering:**
  - A moving average filter is implemented to reduce noise in temperature readings.
  - Outliers exceeding a threshold of 50°C are automatically removed.

- **Voltage Divider:**
  - A voltage divider is used to scale the temperature sensor output to a range suitable for the ADC.

- **Relay Control:**
  - Relay control for the bulb includes a pull-up resistor for proper functioning.
