# Obstacle Avoidance Drone

Capstone Group: PN13

Name of the group members: Aryan Chahardovalee, Anna Wang, Kris Zhou, Rain Zhang, CK Chan 

Name of the Project: Gesture Controlled Vlogging Drone 

Hardware in use:

- Sensor Module: Arduino Nano 33 IoT + 2 Polulu time-of-flight (TOF) sensors + LiPO battery + one front sensor + sensor case + sensor clamp for mounting the sensor on the drone
- Drone: Parrot Anafi Base Drone + extra battery
  - 1080p video
  - 25 minutes flight time
  - For full documentation, refer to useful links
- Huawei Atlas board 200 DK + bootable SD Card (Linux 18.04)
- TP-Link AC750 Wi-Fi Travel Router 
- PC (not included; the PC needs to run Windows with MobaXterm Installed and all the instructions in Section 5 – Quickstart DEMO must be followed) 

Upon receiving the package from Capstone Group 13, the client can jump to **Section 5** to perform a demo. It should be mentioned that as a result of a recent crash the front left-side leg of the drone broke and after capstone team 13 changed the broken leg the drone flies abnormally. The fix for this is to change the current rotor back to the old rotor. Instructions on how to do this are below in **Section 1 - Drone**.  The 3D printed arduino casing is cracked and both Pololu boards (normally containing 6 sensors each) are missing one sensor chip each. We recommend reprinting the 3D case (files provided) and replacing the sensors.  The rest of the sections are included for better understanding the system and to be able to reproduce the system if need be. Pay attention that upon each unsuccessful flight (CRASH), the drone needs to be re-calibrated for best performance (**Section 1 - Drone**). Additionally, the sensor reset button needs to be pushed once to reset the sensor if the connection to the drone is lost (**Section 3 - Arduino**). 

## Section 1 - Drone

After each crash, the drone needs to be recalibrated. Download the FreeFlight6 App from the Apple AppStore and connect to the drone with your phone to see many of the settings that can be controlled through the App. The App can be used to calibrate the drone and the camera as necessary. The specific pre-sets of the drone, such as maximum height and tilt, can also be adjusted as specified in the app. Videos can be viewed from the drone and can also be downloaded to the phone.

![image](https://user-images.githubusercontent.com/73012787/234378615-3873b705-10fa-42dd-a319-7ce3f825cf6a.png)![image](https://user-images.githubusercontent.com/73012787/234378666-62a64dfd-d9de-414c-a6aa-2231867ca8ed.png)
![image](https://user-images.githubusercontent.com/73012787/234378685-a9b81324-2c85-430b-8127-811423a2f753.png)![image](https://user-images.githubusercontent.com/73012787/234378696-f5237dff-5ad4-4f20-936d-da4971729614.png)
![image](https://user-images.githubusercontent.com/73012787/234378714-35267a8c-2fdc-4b12-9cba-425fe0b961b1.png)![image](https://user-images.githubusercontent.com/73012787/234378733-7aef7e69-73e5-425b-802b-4a3db2916fc8.png)
