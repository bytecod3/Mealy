## Mealy - A self-balancing robot



## About
Mealy is a robot build with the aim of making it self-balancing on two wheels, using an MPU6050 IMU and an
STM32F103C8T6 MCU. It is built for the sole reason of learning about control systems specifically the PID control algorithm, and how 
it is and can be applied to real robotics systems. This means that tuning the system, which is a major part of the bulding process,
is considered throughly. 

![3D Model]("./assets/assembly-design.png")

### Motivation 
This robot was inspired by a recent video I stumbled up on youtube (https://youtu.be/-bQdrvSLqpg), where the maker was tuning the  robot's PID gains 
visually, and at the same time using RF to control the robot, with an interface to show the gains and the effects. So I decided to build a cheaper version of the same 
for a start.

### At a glance
Mealy is controlled using an STM32F103C8T6 blue pill development board. The tilt angle is measured using an MPU6050 IMU. 
Wheels are driven using a pair of DC geared motors. The motor driver being used is L293D motor driver. 

### Specs

The following table shows the specifications listed for summary:

|Name | Spec |
|---|---|
|MCU|STM32F103C8T6 Bluepill|
|Inertial Measurement Unit|MPU6050|
|Motors|2 DC geared motors|
|Motor driver|L293D |
|Shape| Hexa-rectangular |
|Width(including wheels)|140 mm|
|Height| 180 mm |
|Control algorithm|PID|

### How it works

There are a few major moving parts on how a this self balancing robot works. 

#### Inertial Measurement Unit


## Features 

## 3D Model 

## Future improvements
