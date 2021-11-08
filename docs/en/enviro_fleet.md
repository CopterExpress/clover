# EnviroFleet

[CopterHack-2022](copterhack2022.md), team **EnviroFleet**.

## Team information

The list of team members:

* Daniel Gu, @danielgu0305@gmail.com, programmer.
* Parthiv Nair, @parthivnair1@gmail.com, programmer.
* Mahmoud Abdelmoneum, @372278@bsd48.org, engineer.
* Sai Nagabothu, @saiteja.nagabothu@gmail.com, hardware.

## Project description

### Project idea

EnviroFleet is a fleet of autonomous drones that coordinate to create 2d maps of natural disasters. This is more efficient, and cost effective for emergency responders than existing solutions that are expensive, not rapidly deployable, and don’t scale to large disasters. By coordinating drones simultaneously, this lends an opportunity to scale our solution much better than existing solutions like manned aircraft or single drone operators. This data can drastically improve disaster response by giving crucial geospatial data that improves spatial awareness and emergency response operation efficacy.

### The potential outcomes

We hope to accomplish disaster mapping with one Clover drone, and use our findings to potentially apply our work to a fleet of coordinated drones. Existing solutions do not optimize this process for disaster responding as they often must create their own waypoints for mapping. We hope to create an interface that allows users to easily outline an area, and our software coordinates the flight pathing automatically. Afterwards, the drone will follow this flight path in a simulated “natural disaster” (likely through testing indoors). The result will be the drone periodically taking images of the environment, and these images will be stitched to create a 2d map.

### Using Clover platform

The Clover platform will be used to primarily control navigation of the drone, as well as optimize the movement and balance. For example, through the use of GPS and ArUIco markers, we can create precise autonomous movement that creates very accurate maps. The Clover Platform also makes it much more efficient for us to rapidly test our prototypes as the ROS framework makes programming than if we were to develop our drone from scratch. Lastly, we believe the Clover simulator will be especially useful for us because it contains so many features of the real drone, which will be very useful for our team when some of us need to work remotely due to distance or COVID restrictions.

In other words, we believe the Clover platform will make development much more efficient and optimized for our team to learn and build, and in turn will allow us to bring EnviroFleet to life.

### Additional information at the request of participants

To help fund EnviroFleet, Daniel and Parthiv have taken the idea to NFTE and won 1st in the PayPay Environmental Justice Challenge. More can be found at https://www.nfte.com/nine-teams-win-more-than-21k-in-nfte-innovation-challenges/.

Parthiv and Sai are part of Oregon’s top robotics teams (Overcharged and RevAmped FTC). Check out this link to see their performance last year!

http://www.ftcstats.org/2020/oregon.html
