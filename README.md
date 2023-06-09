# Autonomous Vehicle
Using the PX4 framework, autonomy was given to a vehicle consisting of DC motors, servo motors, ultrasonic sensors, and Raspberry PI camera. 

The autonomous vehicle can avoid obstacles by detecting the distance from an object with the ultrasonic sensor. 
The camera allows the vehicle to decide where it should steer to avoid the obstacle. For example, if the obstacle were closer to the left of the camera view, the vehicle steers right.

uORB messaging is used for inter-thread/inter-process communication. Any application in PX4 can subscribe to a number of
messages and publish some other messages. For instance, the application (process) that interfaces with the accelerometer sensor in FMU, publishes a uORB message that contains the
sensor data, and other applications that require to use the accelerometer data, subscribe to that message to get the updated values of the sensor.

## To view the written code:
Look in src/examples/hello_world/hello.cpp for the uORB messaging, LED and motor control. <br>
Look in py.pi for the ultrasonic sensor and camera processing, along with the LED and motor control signals.
