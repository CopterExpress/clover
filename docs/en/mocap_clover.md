# Project name

[CopterHack-2023](copterhack2023.md), team **Clover with Motion Capture System**.

<p align="center">
  <img src="../assets/mocap_clover/semi_logo.png"  width="70%" height="70%">
</p>

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

The Clover with Motion Capture System educational document is divided into three main sections outside of the Introduction and Conclusion. Each section and its purpose is listed:

Hardware: The main goal in this section is to educate the reader on the MoCap system hardware and software


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


