# AUTONOMOUS RACING DRONE
[CopterHack-2023](copterhack2023.md),team **DJS PHOENIX**.

## Team Information
![Without bg](https://user-images.githubusercontent.com/93365067/195973101-0bea8e55-ba1f-4f0d-b036-a5c7a04ab51e.png)<br/>
We are the DJS Phoenix, the official drone team of Dwarkadas. J. Sanghvi College of Engineering
The list of team members:

* Shubham Mehta, @Just_me_05, Mentor
* Harshal Warde, @kryptonisinert, Mechanical
* Parth Sawjiyani, @Non_Active, Mechanical
* Soham Dalvi, @devilsfootprint_1973, Mechanical
* Vedant Patel, @VedantMP, Mechanical 
* Harsh Shah, @harssshhhhh, Mechanical
* Lisha Mehta, @lishamehta, Mechanical
* Shubh Pokarne, @Shubhpokarne, Electronics
* Tushar Nagda, @tushar_n11, Electronics
* Deep Tank, @Kraven, Electronics
* Khushi Sanghvi, @Cryptoknigghtt, Programmer
* Harshil Shah, @divine_fossil, Programmer
* Omkar Parab, @Omkar_parab21, Programmer
* Isha Shah, @isha_s_shah, Programmer
* Madhura Korgaonkar, @Madhura221, Programmer
* Shruti Shah, @Shrutishah22, Programmer
* Aditi Dubey, @aditi_0503, Marketing
* Krisha Lakhani, @krishalakhani, Marketing 
* Vividha Jagtap, @vividha_jagtap, Marketing
## Project Description
Since they became more robust in recent years, autonomous drones have been used for a wider range of functions in a variety of commercial enterprises. They are still far from completely utilising their physical potential, though. In fact, to fully assess their environment and have enough time to avoid obstacles, the majority of autonomous drones only fly at low speeds close to hover conditions. Autonomous drone's flight range might be extended by faster and more agile flying, which would also let them avoid dynamic impediments more quickly and navigate more easily in small places. Drones can go across complicated areas like racetracks at amazing speeds, as demonstrated by human pilots. However, in terms of speed, adaptability, and robustness, autonomous drone performance is still far below that of humans, necessitating a significant amount of study and innovation to close this gap.

## Project Idea
Our idea is to build an autonomous racing drone so, our system is composed of five functional groups: Sensor interface, perception, state estimation, planning and control, and drone interface . In the following, a brief introduction to the functionality of our proposed perception, state estimation, and planning and control system is given.
* Perception
  Of the two stereo camera pairs available on the drone, only the two central forward-facing cameras are used for gate detection  and, in combination with IMU measurements, to run VIO. The advantage is that the amount of image data to be processed is reduced while maintaining a very large field of view.
* State Estimation 
  In order to compensate for a drifting VIO estimate, the output of the gate detection and VIO are fused together with the measurements from the downward-facing laser rangefinder (LRF) using an Extended Kalman Filter (EKF) Algorithm . The EKF estimates a global map of the gates and, since the gates are stationary, uses the gate detections to align the VIO estimate with the global gate map, ie, compensates for the VIO drift.
* Planning and control
  The global gate map and the latency-compensated state estimate of the vehicle are used to plan a near time-optimal path through the next N gates starting from the vehicles current state.The path is re-planned every time<br/>
    (i) the vehicle passes through a gate<br/>
    (ii) the estimate of the gate map<br/>
    (iii) the VIO drift are updated significantly, i.e. large changes in the gate positions or VIO drift.

## The Potential Outcomes
Using the Clover Platform, we'll build an autonomous drone which will be utilised for racing. The objective of our challenge is to create a completely autonomous drone that can outperform the best human pilot at navigating a race course using machine vision. Autonomous racing drones present fundamental challenges in real-time state estimation, perception, planning, and control due to the high speeds at which drones must fly to outperform the best human pilots, the difficult visual environments (such as low light, motion blur), and the limited computational power of drones.

## The Clover Platform
The Clover platform will be applied in numerous contexts. OpenCV and ROS for the drone are simulated and implemented using the COEX Clover platform. It aids in the integration of our Raspberry Pi and ROS.

## Additional Information
In 2017, a student committee for DJS Phoenix was formed. In India, our team has participated in a number of contests, including IDRL (fourth overall), IIT BOMBAY TECHFEST (sixth rank), and TECHNOXIAN (second place out of 50 national teams).
In COPTERHACK-2021, our team participated, and we placed eighth internationally. We are back with improved concepts after learning from the previous season.
    
