# Drone-Flight-Controller-Code
This is a flight controller program written in Arduino for a quadcopter. 

The code uses a simple PID controller for controlling motion along the pitch, roll and yaw axes. The PID gain values for the three degrees of motion can be changed under PID settings. The gain values for the roll and pitch are initialized to the same value.

The receiver used has 6 channels and utilizes the digital pins 8-12.
The 4 ESC (Electronic Speed Controllers) utilize the digital pins 4-7

The wire.h library was used for I2C communications with gyroscope. The Gryoscope used is MPU6050.
