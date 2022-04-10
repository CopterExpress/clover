# Obstacle avoidance using artificial potential fields method

[CopterHack-2022](copterhack2022.md), team **Stereo**.

## Team information

The list of team members:

* Denis Konstantinov, @den_konstantinov, engineer and developer.

[The project repository is here](https://github.com/den250400/potential-fields-obstacle-avoidance)

## Project description

### Project idea

<p align="center">
  <img src="https://github.com/den250400/potential-fields-obstacle-avoidance/blob/main/assets/avoidance_sim_demo.gif" />
</p>

Design an obstacle avoidance system for quadcopters with Raspberry Pi 4 onboard computer using artificial potential fields method.

Artificial potential fields method is based on considering quadcopter, obstacles and target point as electric-charged points. Quadcopter and obstacles have positive charge, and target point is assigned with negative charge. This results in quadcopter "attracting" itself to the target point, while being repelled by the same-signed charges of obstacles. Using this analogy, you can compute a safe, collision-free trajectory, which can be executed by the vehicle.

<p align="center">
  <img src="https://github.com/den250400/potential-fields-obstacle-avoidance/blob/main/assets/Traditional-artificial-potentials-path-planning_Q320.jpg" />
</p>

### The potential outcomes

An algorithm which can fly through regions with obstacles with a speed of at least 10 km/h and success rate of at least 95%. In real world, such solution can be used in search-and-rescue mission (might be useful for famous Russian LizaAlert).

We also plan to test this algorithm on the real drone with Clover firmware and Raspberry image. It will use an Intel Realsense D435 depth camera for retrieving geometrical information about the surroundings. We already have a real 6-inch drone with Realsense D435 installed on it, and currently solving problems with local positioning.

### Using Clover platform

The project uses [Clover Raspberry Pi image](https://clover.coex.tech/en/image.html) and [special PX4-based firmware](https://clover.coex.tech/en/firmware.html) modified for easier communication with Raspberry Pi.

