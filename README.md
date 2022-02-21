# Energy-Monitoring-System 
The use of electric machines and motors is essential in almost every
industry. The machines and motors may run into faults due to various
reasons. We have considered following factors that may induce fault in
the machines and provided an embedded solution:
• Current
• Voltage
• Temperature
The embedded system has two nodes, a sensor data acquisition system
and a node that performs calculations and act accordingly. 

developed a FreeRTOS on micro-controller’s that will be
responsible for monitoring of energy consumption and protection of a
motor.
HARDWARE
MOTHER MCU
The mother MCU is STM32F103 Blue Pill. Its specifications are
following:
• ARM Cortex M3
• 2.7 – 3.3V operating voltage
• 64Kb flash memory
• 20Kb RAM
DAUGHTER MCU
The mother MCU is Arduino Uno. Its specifications are following:
• ATmega 328P AVR
• 5V operating voltage
• 32Kb flash memory
• 2Kb RAM
SENSORS
We used 3 sensors to monitor current, voltage and temperature.
• ACS 712 is used to measure current and voltage.
• MPU 6050 is used as a temperature sensor
