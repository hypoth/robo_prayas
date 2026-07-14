# Obstacles Avoiding Robot (From concept to v1.2)

This document details the mathematical framework, calculations, and lookup tables required to make a two-wheel differential drive robot which can execute precise in-place turns, move forward and backwards while avoiding obstacles.


## Simulation
The robot started as simple simulatios on online tool "TinkerCad". A basic version of the robt was made with components available for simulation on TinkerCad. 

### Simulation of the robot
https://www.tinkercad.com/things/lA5BwXk2kSj-diffdrivebotver1/editel?returnTo=https%3A%2F%2Fwww.tinkercad.com%2Fdashboard&sharecode=8Be7qrpKp4Ho1W9cGgfTjN29vAA4K9HrsxYx8px5Kfg

### Simulation Schematics
The schematics for the simulation can be found at [schematicsis](schematics/DiffDriveBot.pdf)

### Robot Schematics
Actual robot is constructed from the same [schematicsis](schematics/DiffDriveBot.pdf) . 

The key difference from schematics is 
* Robot uses LN298 D dual motor power driver.
* 12 V Li Ion Battery pack is used for making the system.

## Robot Construction
The robot consists of following main parts.
* Acrylic Chassis, Track Width ~ 100mm 
* Castor Wheel 
* Yellow Wheel dia 65mm
* Arduino Uno 
* LN298 D dual motor dirver
* Li Ion, battery pack (3 nos, 3.7 V)
* DC motor (max 100 rpm)
* connection wires and jumpers. 

For the prototype, the jumper wires, zip ties are used which do not provide very rigid structure. 
For version 1, these provide a good enough platform for prototyping.

## Software 
The Software is devloped using Arduino IDE and libraries available for Arduino Uno. 
Since this is first prototype and given the processing capabilites of Arduino Uno, a very basic loop for obstacle avoidance is implemented.

## Basic Flow
The simple flow of sense, calcuate and act paradigm of this robot is depicted in following form. 
[Flow Chart](diagram/flowChart.svg)

## Arduino Sketch.
The various versions of [Arduino sketch](Arduino/bot.ino) 


## Learnings

### Mounting various components
* Loose connections
    * Jumper wires are not best options. 
* Flimsy Chassis cuttings, leading to cracking of acrylic. 
* Bobbling of wheels
    * Balancing of wheels is very difficult. 
    * The robo drifts to its side and does not maintain steady path. 
    * Needs constant adjustments, which are not implemented in this version. 
* Mounting of SR04 Ultrasonic sensor of Servo
    * The longitudinal axis of robot and Servo motors zero position does not match all the time. 
    * During gluing of sensor a axis mismatch occurs, which was corrected by adding fixed bias. 
    * This results in some loss of movement at the extreme ends, but is a good compromise.  
* Battery Case mounting
    * Spring loaded battery casing is difficult to handle and causes unexpected shift in variousl components. 
        * used zip-ties but the movement in components (dc motor) still remains. 
        * switch for battery is required, which is not part of this version/kit.
        
### Calibration is necessary, but can not solve for multiple dynamic problems.
* Battery Voltage Changes. 
    Voltage drop causes rpm of motor to change for same PWM command. So the precise contorl becomes difficult.
    * Active feedback is better option for precise control

## Future Enhancements
   * Soldering of components instead of jumper wires.
   * Proper harness and clamps for Motors.
   * Balancing of wheels.
   * Feedback control for motor speed control.
        * encoder based.
   * Add enhanced processing power to implement
        * autonomous navigation algorithms.
        * Localization support.  
 
