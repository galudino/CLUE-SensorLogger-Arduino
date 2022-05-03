# CLUE-SensorLogger-Arduino
Firmware for the Adafruit CLUE (nRF52840 Express) - sensor tests with Bluetooth LE capability

## Notes
This repository is currently under construction.

### Starting off with `bluefruit_playground.ino`
I have decided to start off [main.cpp](src/main.cpp) by using the `bluefruit_playground.ino` example Arduino sketch,<br>
as my main goal is to write a mobile client application for the Adafruit CLUE anyway.

### PlatformIO for building
In short (will add more to this later):
- Get Visual Studio Code
  - Get the PlatformIO extension
- Build the repository

[CLUE-SensorLogger-Arduino](https://github.com/galudino/clue-sensorlogger-arduino) uses [PlatformIO](https://platformio.org) as a build solution.<br>

I am using [PlatformIO](https://platformio.org) with [Microsoft Visual Studio Code](https://code.visualstudio.com).<br>

Learn more here: [https://docs.platformio.org/en/latest//integration/ide/vscode.html](https://docs.platformio.org/en/latest//integration/ide/vscode.html)

#### Why?
I wanted to develop, debug, and build using my preferred tools.<br>
Originally, I was using [Arduino-CMake-Toolchain](https://github.com/a9183756-gh/Arduino-CMake-Toolchain),<br>
but it was tough to set up.<br>

Apparently, the CMake/Arduino solutions had gone through several iterations,<br>
and after coming across this [post](https://github.com/arduino-cmake/Arduino-CMake-NG/issues/100), I decided to give up on CMake playing nicely with Arduino, and use [PlatformIO](https://platformio.org).<br>
(not to mention, it was recommended by the OP)
