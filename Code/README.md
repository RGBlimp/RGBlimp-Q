# RGBlimp-Q Code

## ESP32-Arduino
ESP32 is a low-cost, low-power system on a chip microcontrollers with integrated Wi-Fi and dual-mode Bluetooth, which could be programmed through Arduino IDE. 

### Installing Arduino-ESP32 Support
Stable release link:
```
https://espressif.github.io/arduino-esp32/package_esp32_index.json
```
To start the installation process using the Boards Manager, follow these steps:
* Install the current upstream Arduino IDE at the 1.8 level or later. The current version is at the arduino.cc website.

* Start Arduino and open the Preferences window.

* Enter one of the release links above into Additional Board Manager URLs field. You can add multiple URLs, separating them with commas.

* Open Boards Manager from Tools > Board menu and install esp32 platform (and do not forget to select your ESP32 board from Tools > Board menu after installation).

* Restart Arduino IDE.

###### * See [Installing Guides](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html) for more details on how to install the Arduino ESP32 support. 

### Installing Rosserial Arduino Library
Rosserial is a protocol for wrapping standard ROS serialized messages and multiplexing multiple topics and services over a character device such as a serial port or network socket. 
The rosserial arduino library support for Arduino compatible boards including ESP32. 

To install rosserial arduino library for Arduino IDE, follow these steps:
* Open the **Library Manager** from the IDE menu in Sketch -> Include Library -> Manage Library. 
* Search for "rosserial", and install. 

###### * See [Installing Guides](https://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup) for more details on how to install the rosserial arduino library for Arduino IDE. 

## ROS Package
This ROS package has been tested with ROS Melodic under Ubuntu 18.04. 
```
# Copy rgblimp_ros to catkin workspace

cd ~/catkin_ws/
catkin_make

source ~/catkin_ws/devel/setup.bash

roslaunch rgblimp_ros rgblimp_keyboard.launch
roslaunch rgblimp_ros rgblimp_tuner.launch
```