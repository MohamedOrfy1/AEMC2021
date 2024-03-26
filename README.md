## All Egypt Micromouse Competition - Winning Code

This repository contains the Arduino code used for the winning robot in the All Egypt Micromouse competition. The code controls the robot's movement and navigation within a maze, utilizing sensor data and a compass for orientation.

### Hardware Requirements

* STM32 Bluepill
* Compass module (e.g., HMC5883L)
* Three distance sensors (e.g., VL530X)
* Two motors with encoders
* Motor driver board

### Code Overview

The code is structured as follows:

* **Modules.h:** Header file containing function prototypes and global variables.
* **Main Code:**
    * **setup():** Initializes the I2C communication for the compass, sets up the sensors and motor driver, and calibrates the compass using the start switch.
    * **loop():** Continuously reads sensor data and adjusts the robot's movement based on the following logic:
        * **Calibration:** Checks if the robot is calibrated. If not, it waits for the start switch to be pressed and sets the initial compass reading as the reference angle.
        * **Sensor Readings:** Reads the values of the three distance sensors (left, front, right).
        * **Navigation:** Implements a simple navigation algorithm based on sensor readings:
            * If the left sensor detects a wall within a close distance, turn hard right.
            * If the front sensor detects a wall within a medium distance, move forward while adjusting direction based on the left and right sensor readings to stay centered in the corridor.
            * If the right sensor detects a wall within a close distance, turn hard left.
            * If no walls are detected within the specified thresholds, turn hard right (this behavior might need adjustment depending on the maze layout and competition rules).

### Additional Notes

* The code includes commented-out sections related to PID control and path saving, which were potentially explored during development but might not be actively used in the final version.
* The specific sensor thresholds and turning angles might need to be adjusted based on the robot's hardware and the characteristics of the competition maze.
* The code can be further enhanced by implementing more sophisticated navigation algorithms, such as wall following, flood fill, or dead reckoning.

This code provides a basic framework for micromouse navigation and serves as a starting point for further development and optimization.
