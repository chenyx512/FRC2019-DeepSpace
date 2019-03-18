# FRC2019-Official

*This is the official codes for 2019 FRC competition.*  
updated 20190316 --chenyx  

## Official
*The code has been documented and cleaned*  
This is the official code that runs on the robotRio
* RobotState  
This copies 254's code and implements a localization system based on two wheel encoder and a gyro, with a map to interpolate past history
* vision  
This class is in charge of getting the data from raspberrypi using NetworkTable and calculate the target positions.
It also tries to update the position of the locked target.
* HUD  
This class displays the recognized/locked targets, and the current status of the hatch slide on the driving camera.
* Commands/AutoRun  
This command is the auto command that deliver/get the hatch to/from the locked target.
On start, it chooses the most central target on the camera as the locked target.
It controls the turning speed based on PID on error angle, and it should be changed to a arc to target.

## rpiVision
*The code has been documented and cleaned*  
This is our vision code in java that runs on the raspberrypi, with WPI image 4
* ProcessThread  
Every processThread takes in a Mat picture, from which it extracts all the target and post their info on NetworkTable

## unused
*this part is NOT documented or cleaned*  
This is what I have tested but not used in the actual competition, which includes:
* MechPosition  
This is an experimental localization system based on four wheel encoders on a mech drive system, which we decided not to use this year.
* Trajectory  
This is the trajectory library that I implemented from 254's ideas using 254's library.
It can generate and timestamp a 5-degree spline fitting two poses and feedforward to follow.
