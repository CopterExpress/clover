# clover_blocks

Blockly programming support for Clover.

<img src="screenshot.png" width=700>

See user documentation at the [main Clover documentation site](https://clover.coex.tech/en/blocks.html).

Internal package documentation is given below.

## Frontend

The frontend files are located in [`www`](./www/) subdirectory. The frontend application uses [`roblib.js`](http://wiki.ros.org/roslibjs) library for communicating with backend node and other ROS resources.

## `main.py` node

`main.py` is the blocks programming backend. This node names itself `clover_blocks`, so this namespace is used for all the private topics and services.

### Services

* `~run` ([*clover_blocks/Run*](srv/Run.srv)) – run Blockly-generated program (in Python).
* `~stop` ([*std_srvs/Trigger*](http://docs.ros.org/melodic/api/std_srvs/html/srv/Trigger.html)) – terminate the running program.
* `~store` ([*clover_blocks/load*](srv/Store.srv)) – store a user program (to `<package_path>/programs` by default).
* `~load` ([*clover_blocks/load*](srv/Load.srv)) – load all the stored programs.

### Parameters

* `programs_dir` (*string*) – directory for user programs.

### Topics

#### Published

* `~running` ([*std_msgs/Bool*](http://docs.ros.org/melodic/api/std_msgs/html/msg/Bool.html)) – indicates if the program is currently running.

#### Generated code topics

The following topics are published from the code, generated at the frontend side.

* `~block` ([*std_msgs/String*](http://docs.ros.org/melodic/api/std_msgs/html/msg/String.html)) – current executing block (maximum topic rate is limited).
* `~error` ([*std_msgs/String*](http://docs.ros.org/melodic/api/std_msgs/html/msg/String.html)) – user program errors and exceptions.
* `~prompt` ([*clover_blocks/Prompt*](msg/Prompt.msg)) – user input request (includes random request ID string).

This topic is published from the frontend side:

* `~prompt/<request_id>` ([*std_msgs/String*](http://docs.ros.org/melodic/api/std_msgs/html/msg/String.html)) – user input response.
