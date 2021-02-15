^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package clover
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.21.1 (2020-11-17)
-------------------
* simple_offboard: fix checking kill switch state
* led: change default number of leds to 72
* simple_offboard: correctly check manual control timeout, separate it from kill switch check
* simple_offboard: detect killswitch when auto_arm is set (`#280 <https://github.com/deadln/clover/issues/280>`_)
  * simple_offboard: subscribe to manual control
  * simple_offboard: read check_kill_switch parameter
  * simple_offboard: check kill switch status
  * Fixes
* led.launch: add led_count and gpio_pin arguments (`#279 <https://github.com/deadln/clover/issues/279>`_)
  * led.launch: add led_count and gpio_pin arguments
  * docs: new args in led.launch
* main_camera.launch: add forward and backward camera shortcuts
* led: change default low battery threshold to 3.6
* Implement block programming (using Blockly) (`#272 <https://github.com/deadln/clover/issues/272>`_)
  * Clover Blockly: add first blocks set
  * Adjust Blockly settings
  * Fix get_position output type
  * Add screenshot
  * Rename readme.md to README.md
  * Resize screenshot
  * Add package.xml
  * Little change
  * Fixes
  * Add python_compressed to blockly
  * Implement some of the Clover blocks in Python
  * Make Python indentation 4 spaces
  * Fixes to Python blocks implementation
  * Implement set_velocity block in Python
  * Implement wait_arrival block in Python
  * Fix indentation in Python implementation of blocks
  * Fix
  * Fix land_wait template
  * Set reserved words in Python
  * Change default frame_id to aruco_map in get_position block
  * Fix
  * Move blocks definitions to blocks.js
  * Get rid of missing favicon error
  * Simplify navigate
  * Rearrange layout, add tabs
  * Generate Python code
  * Small style change
  * -console.log
  * Code style
  * Use modules
  * Move modules to the header
  * Correct order for ROS definitions + generating "backend" code
  * Fix rangefinder_distance block
  * simple_offboard: commands to change only yaw and yaw rate
  * Implement set_yaw block
  * Start working on Blockly documentation
  * Implement print block with a topic
  * Unneeded code
  * Little fixes
  * Fix indentation
  * Fixes
  * Fix wait_arival, get_distance
  * Implement running Blockly programs, implement prompt block, fixes
  * Add land button
  * Little change
  * Fix reserved words + little fixes
  * +x for main.py
  * Simplify run button
  * Auto-save and load workspace
  * Make land button work
  * Handle exceptions
  * Minor change
  * Add help URL for blocks
  * Fix
  * Implement arrived block
  * Mark blockly and highlight.js as linguist-vendored
  * Add forgotten CMakeLists.txt
  * Add wait checkbox to set_yaw block
  * Disable run button when disconnected
  * Add message and service files
  * Add some comments
  * Add tooltip to some blocks
  * Implement GPIO blocks
  * Don’t latch print message to prevent duplication
  * Prevent duplication prompts
  * Add ROS init code to backend code anyways
  * Make GPIO blocks color a constant
  * Minor fix
  * More correctly update blocks on input value changes
  * Minor fixes
  * Remove unneeded readonly attribute
  * Add marker ID shadow blocks to toolbox
  * Add lacking reserved words
  * Fix frame id generation for complex marker id expressions
  * Consider frame_id in set_yaw block
  * Shorten ros module import
  * Implement stop service
  * Disable and enable run button correctly
  * Don’t print KeyboardInterrupt exceptions
  * Put notifications to notifications element
  * Add 'running' mark
  * Disable signal in backend python code
  * Sleep a little bit to let rospy initialize publishers
  * Remove accidental code
  * Make ROS namespace and private namespace constants
  * editorconfig-lint: don’t check Blockly code
  * Use private namespace constant in Python generator
  * Implement ~running topic to display current program status more robustly
  * Make navigate tolerance and sleep time constants
  * Make set_leds and and set_effect services proxies persistent
  * Replace a number with constant
  * Limit ~block topic publishing rate
  Otherwise messages get queued making the frontend to freeze
  * Improve internal documentation
  * Append 'map' to frames list
  * Return degrees in get_attitude block
  * Move getting yaw in a separate block
  * Improve block tooltips
  * Add some more files to editorconfig-lint excludes
  * Add get_yaw block to toolbox
  * Implement get_time block
  * Implement ~store and ~load services for storing user programs
  * Set auto_arm only in take_off block
  * Minor CSS fixes
  * Make 'Python' tab textarea-like
  * Implement saving and loading programs
  * Adjust styles
  * Retrieve only .xml files in load service
  * Forgotten code
  * Documentation on store and load services
  * Add some examples
  * Add blocks programming arg to launch file
  * Update docs
  * Add package’s dependencies
  * Add dependency
  * Add title to select
  * Fix syntax
  * Minor fix in docs
  * Add forgotten roslib.js
  * Run user program in the same process
  * Use print function for print block in Python 2
  * Add variables example
  * Fix url
  * Add functions example
  * Fix set_servo block
  * Fix gpio_read block
  * Update blocks screenshot
  * Update docs
  * Update docs
  * Fix set_effect block
  * Minor fix in example
  * Add setpoint block, remove set_velocity from toolbox
  * Remove unused modules
  * Unused variable
  * Add English article skeleton
  * Clarify backend node link error
  * Remove unused variable
  * Update documentation
  * Fix link to documentation
  * Add Blockly logo
  * Update English article
  * Add Blocks programming link to the main page
  * Minor change
  * Add catkin_install_python to CMakeLists.txt
  * Make navigate tolerance and sleep time configurable
  * Add minor todo
  * Add blockly examples directory to editorconfig-lint excludes
  * Rename main node to clover_blocks
  * Add a warning to the old blocks programming article
  * Fix editorconfig-lint exclude
* simple_offboard: set main_camera_optical reference frame to map
* Remove info clover.service as it's only for Raspbian
* Change running section in clover's readme
* simple_offboard: Ensure quaternion initialization
* simple_offboard: relax position_msg timestamp update rules (`#264 <https://github.com/deadln/clover/issues/264>`_)
  * simple_offboard: Relax position_msg timestamp update rules
  * Little code style change
  Co-authored-by: Oleg Kalachev <okalachev@gmail.com>
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
* roswww_static: infrastructure for web-based Clover plugins (`#230 <https://github.com/deadln/clover/issues/230>`_)
  * Package for generating static web sites for ROS
  * rosstatic: add CMakeLists.txt
  * rosstatic: utilize rospkg, store static directory in ROS_HOME
  * rosstatic: default_package param
  * rosstatic: fix URLs in docs
  * clover.launch: make clover the default package for www
  * Unused import
  * Rename rosstatic to roswww_static
  * Fixes
* Put CATKIN_IGNORE file to some directories
* selfcheck.py: don’t fall when ROS_HOSTNAME is not set
* optical_flow: Use functional-style parameter fetching
* optical_flow: Pass nodelet callback queue to TransformListener
* optical_flow: Use cv::Mat(std::vector, bool) ctor for dist_coeffs\_
* clover: Update ros3djs, THREE.js
* clover: Use saner min marker perimeter rate
* Change camera calibration name to main_camera_optical
* Merge pull request `#227 <https://github.com/deadln/clover/issues/227>`_ from goldarte/target-system-id
  Add fcu_sys_id argument to clover.launch
* Merge pull request `#216 <https://github.com/deadln/clover/issues/216>`_ from CopterExpress/new-camera-calib
  Average camera calibration
* Update documentation links
* Continue renaming to Clover
* Add fcu_sys_id argument to clover.launch and mavros.launch to set up target_system_id parameter in mavros
* Fixes
* Move manual installation and running to clover/readme.md
* clover: Add required OpenCV libraries
* selfcheck.py: correctly detect unset aruco_detect/length
* selfcheck.py: don’t fall down on unset known_tilt
* Add ROS service for executing shell commands (`#210 <https://github.com/deadln/clover/issues/210>`_)
  * Add ROS service for executing shell commands
  * Show image version on index web page
  * Add test for exec service
  * Add shell node to clover.launch
  * Remake exec handling, consider exit code and exec failures
* simple_offboard: add position setpoint frame
* led: don’t crash on incorrect mode value
* Camera info resolution matching camera resolution is not necessary with auto rescaling
* clover.launch: enable optical flow by default
* clover: Update roslib.js and ros3d.js
* Keep only one calibration file
* Camera calibration: set principal point strictly to the center
* optical_flow: parameter for setting ROI in radians (`#213 <https://github.com/deadln/clover/issues/213>`_)
  * optical_flow: parameter for setting ROI in radians
  * Compatibility with old OpenCV
* Remove unused fpv_camera.launch
* Simplify camera orientation setting (`#204 <https://github.com/deadln/clover/issues/204>`_)
  * main_camera.launch: simplify camera orientation setting
  * Fix camera transforms
  * Move camera transform description closer to transform tempalte
  * orientation => direction
  * Fix
* aruco.launch: set default corner refinement method to 2 (contour)
* selfcheck: Print board model (/proc/device-tree/model) (`#209 <https://github.com/deadln/clover/issues/209>`_)
* selfcheck: Fix typo in constant name
* Rename package to clover (`#179 <https://github.com/deadln/clover/issues/179>`_)
* Contributors: Alexey Rogachevskiy, Arthur Golubtsov, Oleg Kalachev

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
