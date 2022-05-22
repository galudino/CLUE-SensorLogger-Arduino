# CLUE-SensorLogger-Arduino
Firmware for the Adafruit CLUE (nRF52840 Express) - sensor tests with Bluetooth LE capability

## Notes
This repository is currently under construction.

### Starting off with `bluefruit_playground.ino`
I have decided to start off this project by using the `bluefruit_playground.ino` example Arduino sketch,<br>
as my main goal is to write a mobile client application for the Adafruit CLUE anyway.
The original has been cleaned up/refactored a bit to taste.

As of now, this firmware works with Adafruit's Bluefruit Playground application ([https://github.com/adafruit/Bluefruit-Playground](https://github.com/adafruit/Bluefruit-Playground)), and I intend to write my own mobile client for iOS -- mainly for my own educational/personal pursuit.

### `arduino-cli` to build

You will need to have `arduino-cli` installed in your command line/Terminal to build the project with these instructions. If you already have the Arduino desktop application/Arduino IDE, you can open the [`SensorLogger.ino`](SensorLogger/SensorLogger.ino) file in the IDE and compile/upload from there.

Otherwise, read below for instructions on building with `arduino-cli`.

- Start by getting your CLUE's `port` and `FQBN`:
  ```
  arduino-cli board list
  ```

  - (result):
  ```
  Port                            Protocol Type              Board Name    FQBN                        Core          
  /dev/cu.BLTH                    serial   Serial Port       Unknown                                                 
  /dev/cu.Bluetooth-Incoming-Port serial   Serial Port       Unknown                                                 
  /dev/cu.usbmodem231101          serial   Serial Port (USB) Adafruit CLUE adafruit:nrf52:cluenrf52840 adafruit:nrf52
  /dev/cu.usbmodem231401          serial   Serial Port (USB) Unknown    
  ```

- To build/upload the project to the CLUE, we will need:
  - `port` 
    - (`dev/cu.usbmodem231101`) in this example, may be different on your machine.
  - `FQBN`
    - (`adafruit:nrf52:cluenrf52840`) is what you will also use to compile this project.
  - `project directory`
    - (`SensorLogger`) is what you will also use to compile this project.

- Next, set your CLUE in DFU mode by pressing the RESET button twice.
- Finally, compile and upload the project.
  - Generally, to build and compile in one command:
  ```
  arduino-cli compile [project directory] -b [FQBN] -u -p [port]
  ```
  - Using our `port`, `FQBN`, and `project directory`:
    ```
    arduino-cli compile SensorLogger -b adafruit:nrf52:cluenrf52840 -u -p /dev/cu.usbmodem231101
    ```
    - Remember that the `port` parameter is based off of this example, and you'll want to run `arduino-cli board list` on your own to see what the `port` value is on your machine.
- The built firmware should now be uploaded and running on your CLUE.
- You can verify this by using a Bluetooth device sniffer (or your mobile phone's Bluetooth preferences) to see if your CLUE is advertising.
- For ease of use, you can edit the [`upload`](upload) script and provide arguments for the `FQBN` and `PORT` variables from `arduino-cli board list`, so that way, you need not type out the full command to compile and build each time when needed.
  - Just run the following in the terminal, after editing the `FQBN` and `PORT` variables:
    ```
    ./upload
    ```
    - Examples for `FQBN` and `PORT` were provided, but they might not work on your system.
