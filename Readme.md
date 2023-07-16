[![License: MIT](https://img.shields.io/badge/license-MIT-green.svg)](https://github.com/NoX209/YDLidarX4/blob/master/LICENSE)

# Library for using a YDLidarX4 on a ESP32

## General
This lib reads a YDLidarX4 via serial interface an provide the corrected angle and distance values.

## State
* receive and decode scan data :white_check_mark:
* receive and decode health date :x:
* receive and decode device info data :x:
* receive and decode non scan responses while in scanning state :x:
* handle timeout on lidar data :white_check_mark:
* capture time on received index packet :x:
* periodicaly check device heath :x:

## Usage
This library supports the following devices :
* YDLidarX4

# License
MIT - See file "LICENSE"
