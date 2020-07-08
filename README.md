# bluecougar_x_ros
A ROS C++ wrapper for Matrix Vision USB 2.0 vision camera, mvBlueFOX-MLC series.
I tested it with ROS melodic + linux Ubuntu 18.04 LTS. A tested camera model is exactly mvBlueFOX-MLC200wC/G.

## Feature
I construct this code for 'hardware sync. by external trigger signal'. This node supports hardware trigger mode. 
We use a digIn0+ pin on the camera 12-pin outlet as a trigger pin. 
We provide trigger signal (0~5V digital, 0~0.3V low / 3.0V~12.0V high) from the Arduino Mega digital signal.

## Pre-requisite
Before using, camera driver (mvIMPACT) need to be installed. It can be found at the manufacturer site. (Matrix Vision).

## Camera hardware configuration
none.


## Questions & issues
Please contact to e-mail (hyun91015@gmail.com).
