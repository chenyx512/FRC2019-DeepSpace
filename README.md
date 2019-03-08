# FRC2019-Official

*This is the official codes for 2019 FRC competition.*
*docs NOT finished*
updated 20190308

## Official
This is the official code that runs on the robotRio
### RobotState
This copies 254's code and implements a localization system based on two wheel encoder and a gyro, with a map to interpolate past history
### vision
This class is in charge of getting the data from the raspberrypi using the NetworkTable and then calculate the target position.
### HUD
This class displays the recognized/locked targets on the driving camera
## rpiVision
This is our vision code in java that runs on the raspberrypi, with WPI image 4
## unused
This is what I have tested but not used in the actual competition, which includes:
### MechPosition
This is a localization system based on four wheel encoders
### Trajectory
This is the trajectory library that I copied from 254.
It now can generate and timestamp a 5-degree spline fitting two poses and feedforward to follow.
