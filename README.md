# AMT213D auto encoder on Arduino Mega

## Description
A program to read and process angle information from the AMT213D position encoder. The AMT213D is a high-resolution encoder ideal for various applications, including robotics, automation, and motion control.

## Table of Contents
- Hardware Setup
- Installation
- Usage

## Hardware Setup
- Controller: Arduino Mega 2560
- Encoder: AMT213D-V
- Connection: Serial communication using RS485 module
  
## Installation
Flash the ino file using anz IDE. I used normal Arduino IDE.

## Usage
Angle can be read on a serial monitor (on PC)


## Reference and footnote
   - This was made with reference to a thread by shmadam 
     on arduino forum. https://forum.arduino.cc/index.php?topic=557964.0
   - Made for AMT213D-V to work with Arduino Mega 2560. Code can be re-adjusted to work with Uno or nano
     by choosing not to use serial monitor for outputs. Maybe use i2c to send data to a raspberry pi.
     
## Contributing
We welcome contributions! If you have ideas for improvements or new features, please fork the repository and submit a pull request. For any issues, feel free to open an issue in the GitHub tracker.

## License
This project is licensed under the MIT License - see the LICENSE file for details.

Enjoy!
