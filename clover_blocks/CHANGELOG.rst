^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package clover_blocks
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.21.1 (2020-11-17)
-------------------
* blocks: add set_duty_cycle block
* docs: add gpio info to block article
* blocks: fix units in set_servo tooltip
* blocks: add led_count block
* blocks: fix gpio blocks indentation
* blocks: change default z to 1 in aruco-marker example
* blocks: fix set_leds with color-typed argument
* blocks: add yaw_tolerance parameter
* blocks: add confirm_run parameter for disabling running confirmation
* blocks: treat cancel in prompt as empty string
* Add code headers
* blocks: remove unused code
* blocks: add print-range example
* blocks: add flip example
* blocks: fix setpoint block
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
* Contributors: Oleg Kalachev

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
