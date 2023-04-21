# Project name

[CopterHack-2023](copterhack2023.md), team **Clover with Motion Capture System**.


<div align="center">
  <a href="https://www.youtube.com/watch?v=jOovjo0aBpQ&t=4s&ab_channel=SeanSmith"><img src="../assets/mocap_clover/semi_logo.png" width="70%" height="70%" alt="IMAGE ALT TEXT"></a>
</div>


## Educational Document
My Gitbook, with detailed step by step analysis of the proposed project during the CopterHack 2023 competition can be found:

## Introduction
Aerial robotics has become a common focus in research and industy over the past few decades. Many control developments in research require a controlled test environment to isolate certain chanarteristics of the system for analysis. This typically takes place indoors to eliminate unwanted disturbances allowing results to be more predictable. Removing localization and pose feedback concerns can be accomplished with motion capture (MoCap) systems that track unmanned aerial vehicles (UAVs) pose with high precision as stated:

https://optitrack.com/applications/robotics/#:~:text=Exceptional%203D%20precision%20and%20accuracy&text=OptiTrack's%20drone%20and%20ground%20robot,error%20less%20than%200.05%C2%B0.
OptiTrack’s drone and ground robot tracking systems consistently produce positional error less than 0.3mm and rotational error less than 0.05°.

This enables researchers to study the dynamics and behavior of UAVs in different environments, evaluate their performance, and develop advanced control algorithms for improved flight stability, autonomy, and safety. Research facilities around the world tend to built research drones from the ground up using off-the-shelf compnenets with open source platforms such as PX4. While the end goal is the same, involving the transfer of pose feedback to the flight controller along with high level commands, the platforms and methods can vary significantly depeding on factors such as onboard and offboard computing frameworks and data transfer methods. Many developers have a detailed background and understanding of the theoretical components of their research, however, adapting hardware implementation methods to their on platform such as sensor feedback and sensor fusion is not obvious. The purpose of this project is to provide detailed documentation on integrating the Clover platform with the MoCap system along with examples to farmiliarize users with the hardware, sensor fusion, high and low level controller development, and trajectoy tracking.

## Educational Document
In this article, we will provide an overview of motion capture systems for tracking UAV pose in research applications, highlighting their significance, advantages, and potential impact on the field of UAV control development.

### Document structure

The Clover with the Motion Capture System educational document is divided into three main sections outside of the Introduction and Conclusion. Each section and its purpose are listed:

Hardware: The main goal in this section is to educate the reader on the MoCap system hardware and software. This can be further divided into several steps including camera placement, marker plcement, and system calibration. A summary of the process is provided:

1. Camera Placement: Position the motion capture cameras in strategic locations around the area where the UAV will be flying. The number of cameras and their placement will depend on the size of the area and the desired capture volume. Typically, cameras are placed on tripods or mounted on walls or ceilings at specific heights and angles to capture the UAV's movements from different perspectives.
2. Marker Placement: Attach OptiTrack markers to the UAV in specific locations. OptiTrack markers are small reflective spheres that are used as reference points for the motion capture system to track the UAV's position and movements. The markers are typically attached to the UAV's body, wings, and other relevant parts in a way that ensures they are clearly visible to the motion capture cameras from multiple angles. AN example placement on the Clover is shown in the educational document.
3. System Calibration: Perform system calibration to establish the spatial relationship between the cameras and the markers. This involves capturing a calibration sequence, during which a known pattern or object is moved in the capture volume. The system uses this data to calculate the precise positions and orientations of the cameras and markers in 3D space, which is crucial for accurate motion capture.
4. Testing and Validation: After setting up the cameras and markers, perform test flights with the UAV to validate the accuracy of the motion capture system. Analyze the captured data to ensure that the UAV's movements are accurately captured and that the system is functioning correctly.
5. Fine-tuning: Fine-tune the motion capture system as needed based on the test results. This may involve adjusting camera angles, marker placements, or calibration settings to improve the accuracy and reliability of the system.
6. Data Collection: Once the motion capture system is properly set up and calibrated, you can start collecting data for your UAV research. The system will continuously track the positions and movements of the markers on the UAV in real-time, providing precise data that can be used for various analyses and experiments.
7. Data Analysis: Analyze the captured data using appropriate software to extract relevant information for your UAV research. This may involve tracking the UAV's position, velocity, acceleration, orientation, and other parameters, and analyzing how they change over time or in response to different conditions or inputs.

Overall, setting up a motion capture system for UAV research requires careful planning, precise marker placement, accurate system calibration, and thorough validation to ensure accurate and reliable data collection for your research purposes.

Data Transfer: With the data aquired from the MoCap system, the main goal in this section is to transfer it to the raspberry Pi onboard the Clover and then remap it to the flight controller/PX4 for control. A summary of the steps are listed:

1. Data Acquisition: The motion capture system continuously tracks the position and orientation (pose) of the UAV using markers attached to the UAV and cameras positioned in the capture volume. The system calculates the 3D pose of the UAV in real-time.

2. Data Transmission: The pose data is transmitted from the motion capture system to a Raspberry Pi, which acts as an intermediary for processing and relaying the data to the flight controller onboard the UAV. This can be done using wireless communication protocols such as Wi-Fi, Bluetooth, or other suitable methods.

3. Data Processing: The Raspberry Pi receives the pose data from the motion capture system and processes it to extract the relevant information, such as position and orientation. This may involve parsing the data, converting it into a suitable format, and performing any necessary computations or filtering to obtain accurate pose information.

4. Data Remapping: Once the pose data is processed, the Raspberry Pi maps it to the appropriate format required by the flight controller onboard the UAV. This may involve converting the pose data into the proper coordinate system or units used by the flight controller, and ensuring that the data is in a compatible format for further processing.

5. Data Transmission to Flight Controller: The remapped pose data is then transmitted from the Raspberry Pi to the flight controller onboard the UAV. This can be achieved through a wired or wireless connection, depending on the specific hardware and communication options available on the UAV.

6. Flight Control Update: The flight controller onboard the UAV receives the remapped pose data and uses it to update the UAV's flight control algorithms. The updated pose information can be used to adjust the UAV's flight trajectory, orientation, or other control parameters to achieve the desired flight behavior or control objectives based on the motion capture system feedback.

Closed-Loop Control: The flight controller continuously receives pose feedback from the motion capture system via the Raspberry Pi, and uses it to update the UAV's flight control commands in a closed-loop fashion. This allows the UAV to maintain precise position and orientation control based on the real-time pose data provided by the motion capture system.

Overall, sending pose feedback from a motion capture system to a Raspberry Pi and remapping the data to the flight controller onboard a UAV involves acquiring, processing, and transmitting the pose data in a compatible format to enable real-time closed-loop control of the UAV based on the motion capture system's feedback.

Examples: This section provides two practical examples to help the user better understand the Clover platform, sensr fusion, UAV applications such as trajectory tracking, high level commands, and low level control. A figure-8 high-level trajectory generation example is outlined for both Software in the Loop (SITL) simulations and hardware testing with the CLover platform. Here's a summary of the importance of trajectory tracking for UAV applications:

1. Navigation and Path Planning: Trajectory tracking allows UAVs to follow pre-defined paths or trajectories, which is essential for tasks such as aerial mapping, surveying, inspection, and monitoring. UAVs can be programmed to autonomously follow specific trajectories to collect data or reach designated locations with high accuracy and repeatability.

2. Precision and Safety: Trajectory tracking enables precise control of the UAV's position, velocity, and orientation, which is crucial for maintaining safe and stable flight operations. Precise trajectory tracking allows UAVs to avoid obstacles, maintain safe distances from other objects or aircraft, and operate in confined or complex environments with high precision, reducing the risk of collisions or accidents.

3. Payload and Sensor Performance: Many UAV applications require precise positioning and orientation of onboard sensors or payloads, such as cameras, LiDAR sensors, or other specialized equipment. Trajectory tracking ensures that the sensors or payloads are accurately positioned and oriented as per the planned trajectory, allowing them to collect data or perform tasks with optimal performance and accuracy.

4. Efficiency and Resource Optimization: Trajectory tracking can optimize the UAV's flight path to minimize energy consumption, flight time, or other resources. By following efficient trajectories, UAVs can optimize their flight routes, reduce unnecessary movements, and conserve energy or other resources, which is particularly important for long-duration missions, battery-powered UAVs, or operations in remote or resource-constrained environments.

5. Autonomy and Scalability: Trajectory tracking enables UAV autonomy, allowing them to operate independently without constant operator intervention. This enables UAVs to perform repetitive or complex tasks autonomously, freeing up human operators to focus on higher-level decision-making or supervisory roles. Trajectory tracking also facilitates scalable operations, where multiple UAVs can follow coordinated trajectories to perform collaborative tasks, such as swarm operations or coordinated data collection.

6. Flexibility and Adaptability: Trajectory tracking allows UAVs to adapt their flight paths or trajectories in real-time based on changing conditions or objectives. UAVs can dynamically adjust their trajectories to accommodate changes in environmental conditions, mission requirements, or operational constraints, allowing for flexible and adaptive operations in dynamic or unpredictable environments.

In summary, trajectory tracking is crucial for UAV applications as it enables precise navigation, safety, efficiency, autonomy, and scalability, while optimizing payload performance and adaptability to changing conditions. It plays a fundamental role in ensuring that UAVs can accomplish their missions effectively and safely, making it a critical component of UAV operations in various industries and domains.

The second example shows the user how to implement the adaptive auto-tune module provided by PX4 to tune the low-level controllers or attitude control module. This is a much faster and easier way to tune a real drone and provides good tuning for most air frames. Manual tuning is recommended when auto-tuning dos not work, or when fine-tuning is esential. However, the process is tedious and not easy easpecially for users with limited control background and experience. The Clover airframe provides good enough base settings where auto-tuning can further improve performance depending on the Clover being used. Here's a summary of the importance of low-level controller tuning for UAV applications:

1. Flight Stability and Safety: The low-level controller, typically implemented as a PID (Proportional-Integral-Derivative) or similar control algorithm, governs the UAV's attitude (orientation) and position control. Properly tuning the low-level controller ensures that the UAV remains stable during flight, with accurate and responsive control inputs. This is essential for safe and reliable UAV operations, as it helps prevent undesired oscillations, overshooting, or instability that can lead to crashes or accidents.

2. Control Precision and Responsiveness: Accurate control is crucial for achieving precise and responsive UAV maneuvers, such as smooth trajectory tracking, precise hovering, or dynamic maneuvers. Proper tuning of the low-level controller allows for precise control of the UAV's attitude, position, and velocity, enabling it to accurately follow desired flight trajectories, respond to changing conditions or commands, and perform complex flight maneuvers with high precision.

3. Payload and Sensor Performance: Many UAV applications rely on onboard sensors or payloads, such as cameras, LiDAR sensors, or other specialized equipment. The performance of these payloads can be greatly influenced by the stability and precision of the UAV's flight control. Proper low-level controller tuning minimizes the vibrations, disturbances, or errors in the UAV's flight, which can adversely impact the performance of the payloads, ensuring optimal data collection or task execution.

4. Energy Efficiency and Flight Endurance: UAVs are often powered by limited energy sources, such as batteries, and optimizing their energy consumption is crucial for maximizing flight endurance and operational efficiency. Well-tuned low-level controllers can minimize unnecessary control inputs, smooth out control actions, and reduce oscillations or aggressive maneuvers, resulting in more efficient flight trajectories and reduced energy consumption.

5. Adaptability and Robustness: UAV operations can be subject to varying environmental conditions, payload configurations, or operational requirements. Proper low-level controller tuning allows for adaptability and robustness, enabling the UAV to perform reliably and accurately across a wide range of conditions or mission requirements. Tuning the controller parameters can help account for changes in payload mass, wind conditions, or other external factors, ensuring stable and responsive flight performance.

6. Safety Compliance and Regulatory Requirements: UAV operations are subject to safety regulations and compliance requirements, which may include specific guidelines for control system tuning. Proper low-level controller tuning ensures that the UAV's flight control system complies with safety standards, regulatory requirements, and manufacturer specifications, reducing the risk of accidents or violations.

In summary, low-level controller tuning is crucial for UAV applications as it directly affects flight stability, control precision, payload performance, energy efficiency, adaptability, and compliance with safety and regulatory requirements. It is an essential step in optimizing the performance and safety of UAV operations, ensuring reliable and effective flight control for various applications across different industries and domains.

## Conclusion




## Team information

The list of team members:

* Sean Smith, @ssmith_81, engineer and developer.

## Project description

This project is an educational reference and detailed tutorial on how to setup the OptiTrack Motion Capture (MoCap) system with the COEX Clover platform. 
It gives brief descriptions on the camera and motive software setup with many resourceful links, but it assumes the user has a basic understanding on how to
setup the cameras and motive computer software. MoCap markers allow the MoCap to stream positional data of the Clover therefore marker placement is discussed. 
From there details on how to stream position data from the MoCap to the Clover along with how to configure the Clover; specifically, the Raspberry Pi and PX4 
firmware parameters are discussed. The overall network will be provided as it is the most important part.

At the end, I will provide an interesting example such as a tracking a complex trajectory that any user can implement.

### Project idea

In many research applications highly precise position feedback is required and that is why a MoCap system is popular in this field of robotics. Research papers 
are published detailed around certain topics such as control, path planning, obstacle avoidance and many more although the details surrounding certain hardware 
setups such as with the MoCap system are not provided. There are a few sources that provide help with setting up the MoCap system with PX4 and other specific 
systems but with limited knowledge of how and why steps are made one might not be able to adapt it to their own setup such as with the Clover. That is why this 
project has been created; so that a student or user can follow this tutorial with the COEX Clover and have a working setup with the MoCap and Clover even with 
a limited understanding of software and hardware. The article also provides descriptions on why certain things are done to allow the user the better understand 
the system setup.

I currently have the setup running but now working well. The Clover is unable to follow setpoints with any precision
therefore working through network and software issues seems to be the current stage (I am not sure what exactly is causing this issue actually). I am hoping to 
receive guidance in this area from this project so I can have it working as desired.

### Using Clover platform

The COEX Clover 4.2 kit is used where the MoCap system setup is specific for the Clover platform. It provides useful information for all robotics users interested in 
implementing external sensor feedback although it is specific for Clover owners.

### Additional information at the request of participants
I am a masters student looking to implement this project in my research.


