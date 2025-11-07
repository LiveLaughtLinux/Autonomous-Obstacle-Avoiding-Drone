# Autonomous-Obstacle-Avoiding-Drone
Autonomous Crazyflie 2.1 drone flight with real-time obstacle avoidance using Multiranger sensors. The drone follows a square path, detects nearby obstacles, sidesteps safely, and lands automatically. Built in Python with the cflib library

# Crazyflie Obstacle Avoidance Flight

An autonomous flight demo for the **Bitcraze Crazyflie 2.1** drone using **Python** and **cflib**.  
The drone flies a square pattern, detects obstacles using the **Multiranger deck**, and performs automatic sidestepping to avoid collisions before returning to its planned route.

---

## Features
- Autonomous square flight path  
- Real-time obstacle detection (front & right sensors)  
- Automatic sidestep maneuver and safe re-centering  
- Controlled takeoff, hover, and landing  
- Configurable radio URI for easy setup

---

##  Requirements
**Hardware**
- Crazyflie 2.1 drone  
- Crazyradio PA dongle  
- Multiranger Deck  
- (Optional) Flow Deck for stability

