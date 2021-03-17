# Drones to fight Corona

## Team

- Daria Miklashevskaya
- Yuriy Sukhorukov

Innopolis University, B17-DS-II, B17-RO-I

## Introduction

The world faces the worst pandemic of XXI century, which affects lives and well-being of millions of citizens. To slow down the spread of the disease and give the healthcare system to react, people are obliged to wear masks, however, people sometimes ignore those rules, which puts lives of others under the threat.

There are laws in place to enforce mask wearing, but there is just not enough cops to monitor the situation in every mall, bus station etc and fine the law-breakers. We want to contribute to the solution of this problem, but instead of punishing people, we want to provide them with mask with our super-friendly and cute drone.

## Custom airframe

Since we do not have the Clover drone, we will use an airframe built and designed by us.

The main idea is to use truss structure, because it works well against twisting and warping.

* The main idea is to use truss structure, because it works well against twisting and warping 
<img src="../assets/en/drone_frame.jpg" title="Frame">
* The main advantage of such a system is that it distributes impact between beams and effectively dissipates it. However, engine mounts are not impact-proof, this is because we sacrifice them, but save much more expensive and not-readily-available engine. This is why mounts are quickly-replaceable (only 3 screws) and mate of cheap PLA plastic

The main advantage of such a system is that it distributes impact between beams and effectively dissipates it. However, engine mounts are not impact-proof, this is because we sacrifice them, but save much more expensive and not-readily-available engine. This is why mounts are quickly-replaceable (only 3 screws) and mate of cheap PLA plastic.

The space inside the central rhombus is occupied by the on-board equipment: batteries, PX4 flight controller, Jetson Xavier NX / AGX.

As it is shown on this picture, AGX is mounted on top of the platform, while NX can be mounted on the bottom, and completely protected from any damage by beams and landing legs.

<img src="../assets/en/drone_with_jetson.jpg" title="AGX is safe">

All sensory equipment, like cameras, rangefinder, etc can be easily mounted on the beams with special connectors, which provide flexibility because you can fine-tune camera angle or position before tightening screws and fixing it firmly in place, which is especially relevant for tracker-cameras.

We used two T-265 cameras for visual odometry (fuse them together with Kalman filter). And one D-435 depth camera for both video input for neural net and for map-building (collision avoidance). Also, we use dampers to prevent odometry drift.

* We used two T-265 cameras for visual odometry (fuse them together with Kalman filter)
And one D-435 depth camer for both video input for neural net and for map-building (collision avoidance)
<img src="../assets/en/camera.jpg" title="Begone odometry drift">

Here are the photo of assembled drone:


## Software

Thus far we discussed things which are specific to our custom airframe. Things we are going to discuss next are applicable for Clover drone as well. Our software is containerized so it can be launched on every platform that supports Docker, be it Windows machine, Linux machine, Jetson or Raspberry Pi.

We have split our drone software into two modules:

* Pipeline, which manages communication with ROS.
* Neural net module, which talks with the pipeline via sockets.

The reason for this is simple - ROS supports ony Python 2, and I do not feel like I'm building it with Python 3 because it is a bit troublesome. Our neural net, on the other hand, uses pytorch which is Python 3 only.

But apart from compatibility issue, such an arrangement allows us to run inference module anywhere we want, like in more powerful desktop PC or even somewhere in the cloud.

This means that we can make our drone lighter by excluding heavy on-board computer like Jetson AGX / NX and replacing it with something light like Raspberry Pi. Pipeline image is made as lightweight as possible, so it should be runnable even on really weak computers.

## Neural net

We use 3rd version of YoLo neural network, pretrained on customly-built dataset for 50 epochs.

## Mask release

Detecting people without masks is cool, no doubt.
<img src="../assets/en/masks.jpg" title="Master and his apprentice">


But we want not only to detect them but to give him a mask as well, so, we have built this system that can give a mask to person
<img src="../assets/en/release.jpg" title="Mask release in cad">
<img src="../assets/en/release_Cad.jpg" title="Mask release manufactured">


This device looks like a regular firearm mag, and functions exactly in the same way. Masks can be loaded into containers made out of 20ml syringe barrels.

This device needs further engineering because current iterations are too fragile and unreliable, probably the best solution will be to use linear actuator and push the "casing" out of the action, like in actual firearm.

## Final thoughts

This project will still be useful even when the Coronavirus crisis will be over. It can deliver some small objects, like cosmetics or shaving blades to the customers' door, the task that currently is done by a human courier. This service (with shaving blades), when a guy comes and brings a new set of shaving blades every week is popular in US and UK, so why not try to automate it a bit. The working principle and hardware will remain the same, but software will need an update

Stay safe folks!
