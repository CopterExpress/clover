^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package clover_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.21.1 (2020-11-17)
-------------------
* Add official Clover simulation config (`#254 <https://github.com/deadln/clover/issues/254>`_)
  * clover_description: Add preliminary configs/models
  * clover_description: Use proper models for the drone
  * clover_description: Be more specific about spawn arguments
  * clover_description: Tweak parameters a bit, add collision boxes
  * travis: Add .dae files to the list of ignored by eclint
  * Add clover_simulation package
  * clover: Add Gazebo plugin sources
  * builder: Ignore clover_gazebo_plugins for actual drone
  * clover_gazebo_plugins: Expose include directories for plugins
  This should fix building the unit tests
  * clover_gazebo_plugins: Remove dependency on gazebo_ros
  This should prevent RPi image failing to build.
  * travis, gitattributes: Mark clover_gazebo_plugins as vendored, stop checks
  * clover_simulation: Minor package.xml fix
  * clover_description: Add IMU joint preservation
  Oh, Gazebo, you are ever so very helpful, it's hard to put my appreciation into words! If not for your helpful model simplification, I wouldn't have spent two hours looking through the plugin sources, the urdf sources, trying lots of
  different options for the joints and links, and finally getting an answer from GazeboOverflow or however you've named your Q&A site. How wonderful it is to have an issue that makes you tear your hair out just because you know
  what's better for me!
  * clover_simulation: Add the bare necessities to run a simulation
  * clover_gazebo_plugins: Prevent gazebo from trying to download models
  For some reason the models are no longer available, so Gazebo just spends some time waiting for a timeout.
  * clover_gazebo_plugins: Update Gazebo model database URI
  * clover_simulation: Add script to find and launch PX4
  * clover_simulation: Fix launch file
  * clover_description: Add missing plugins
  * simulation: Re-enable gazebo_ros dependencies
  This will force rosdep to try to install gazebo_ros on the drone,
  but this can be counteracted by --skip-keys rosdep option.
  This does not look reliable, but I could not come up with a better
  solution.
  * builder: Be more resilient about apt-get errors
  * builder: Remove reference to resolve_rosdep
  * clover_description: Update Clover model, change xacro description
  Previous xacro description file was not performing too well, so I went with
  a more sensible route and started changing iris.xacro to use our Clover model.
  * clover_description: Bring back constants.xacro
  * clover_description: Prevent lumping for camera link/joint
  * clover_description: Move near clipping plane further away
  * clover_description: Allow setting width/height for rpi_cam
  * clover_description: Add a Clover model with a camera
  * clover_description: Remove whitespaces
  * clover_description: Add drone+camera spawning .launch file
  * clover_simulation: Add gazebo_ros here as well
  * clover_simulation: Spawn drone with camera by default
  * clover_simulation: Allow specifying data path for px4
  * clover_simulation: Add startup scripts from px4
  Big TODO: Clean them up eventually
  * clover_simulation: Use local data files
  * clover_simulation: Launch clover services by default
  * clover_description: Depend on gazebo_plugins as well
  libgazebo_ros_camera is in gazebo_plugins, so we need that package.
  * clover_description: Fix camera_sensor description
  * clover_description: Fix typo in package.xml
  * clover_simulation: rename sim_gazebo.launch to simulator.launch
  * clover_simulation: Don't look for ROMFS in px4_source_path
  We provide our own, no reason to fail if we can't find the originals.
  * clover_simulation: Remove extra CMakeLists.txt
  * clover_description: Use xacro: namespace for xacro macros
  * clover_description: Fix package.xml formatting
  * clover_description: Better camera defaults
  * clover_description: Add distance sensor
  * clover_description: Add leg colliders
  * clover_simulation: Actually forward vehicle name
  * clover_description: Revert adding additional colliders
  Unfortunately, this breaks physics too much
  * clover_description: Tweak drone physics, make it more bouncy
  * clover_description: Don't spawn the drone inside the floor
  * clover_description: Set rangefinder min range outside drone collider
  * clover_simulation: Set default flow parameters for Clover
  * clover_description: Update Clover 4 model
  * clover_simulation: rename sim_gazebo.launch to simulator.launch (`#233 <https://github.com/deadln/clover/issues/233>`_)
  * clover_simulation: Add workaround for Gazebo crashes in VMware
  * clover_simulation: Ignore .git for now
  * clover: Add "simulated" argument
  * clover_simulation: Start Gazebo early
  * clover_gazebo_plugins: Remove unused files
  * clover_description: Allow turning sensors on and off
  * clover_description: Fix rangefinder creation
  * Remove unneeded stuff and use PX4 from catkin workspace
  * Remove clover_gazebo_plugins
  * Rename arg simulated to simulator
  * clover: Change target names to avoid clashing with PX4
  * Fix
  * clover_simulation: Re-add deleted comments
  * Add loop model
  * loop.material: use tabs instead of spaces
  * loop model: don’t rotate by yaw
  * loop.material: turn on alpha_blend
  * Rename model loop to loop_line
  * Add parquet plane model
  * loop_line: fix description
  * Set alpha_blend for loop_solid material
  * Add square line model
  * Add CATKIN_IGNORE to models directory
  * Add LED strip Gazebo model
  * Add hardcoded URDF LED strip
  * clover_description: Add LED xarco model
  clover_simulation: Implement LED visual plugin and controller
  * clover_simulation: Make led plugin less chatty
  * clover_simulation: Depend on led_msgs
  This should allow the packages to be built in the proper order.
  * clover_simulation: Support building against Kinetic
  * clover_simulation: Don't build plugins if Gazebo is not installed
  * clover_description: Get rid of "constants" file
  * clover_description: Add README
  * clover_simulation: Add README
  * clover_simulation: Make parquet thicker
  Otherwise the rangefinder beam goes right through it.
  * docs: Start working on simulation articles
  * docs: Start working on the simulation overview (en)
  * Add launch-file for PX4 v1.8.2
  * clover_simulation: Disable GPS, use EKF2 by default
  Ideally we should be using LPE, but it is broken in PX4 1.10, and our need for a somewhat working simulator is higher than for a completely correct one.
  * clover_simulation: Add experimental throttling camera
  * clover_simulation: Add note about throttling camera
  * clover_description: Remove unused file
  * clover_simulation: Link against CameraPlugin
  * clover_description: Add option to use throttling_camera
  * Add clover.world
  * clover_description: Add calculated inertial parameters
  * simulator: change default world to clover.world
  * clover_simulation: Start working on ArUco generation script
  Port over aruco_gazebo_gen, add more options.
  Does not modify the world right now.
  * clover_simulation: Make LED plugin less chatty
  * clover_description: Be more ROS-like in script naming
  * clover_simulation: Implement model insertion to the world
  * clover_simulation: Allow specifying output model dir
  * clover_description: Don't use throttling camera by default
  throttling camera is still a work-in-progress, there's no reason to
  enable it by default.
  * clover_simulation: Use proper script name in CMakeLists
  This is what typically happens when I'm rushed.
  * docs: Add instructions for VM setup (en)
  * clover_simulation: Remove extra spaces
  * docs: Describe simulation usage (en)
  * clover_simulation: Remove led_strip
  * docs/assets: Crunch sim image a bit
  * clover: Bump VL53L1X version
  For some reason, 0.0.2 is not installable on x86.
  * docs/simulation: Fix capitalization
  * Remove remnants of clover_gazebo_plugins
  * Remove unneeded Clover 3 model
  * Remove empty.world and asphalt_plane model
  * Remove unused LED strip model
  * Reduce images size
  * Shortened simulator related urls
  Co-authored-by: Oleg Kalachev <okalachev@gmail.com>
* Contributors: Alexey Rogachevskiy

0.15.1 (2019-01-24)
-------------------

0.11.4 (2018-09-19 21:32)
-------------------------

0.11.3 (2018-09-19 08:16)
-------------------------

0.11.2 (2018-09-19 07:04)
-------------------------

0.11.1 (2018-09-19 04:40)
-------------------------

0.10.1 (2018-09-05 21:15)
-------------------------

0.8.1 (2018-07-25)
------------------
