# roswww_static

roswww_static creates a static web directory for your ROS-powered system with symlinks to all the `www` subdirectories found in your ROS packages. This way you can use any external web server (e. g. [nginx](https://nginx.org/), [Monkey](https://github.com/monkey/monkey), [Caddy](https://caddyserver.com)) to serve you static data, in compatible with `roswww` manner.

Note: you should configure your web server to make it follow symlinks.

## Instructions

* Run `update` script and it will generate the symlinks and index file: `rosrun roswww_static update`.
* Point your static web server path to `~/.ros/www`.

You can rerun `update` if the list of installed packages changes.

## Parameters

Parameters are passed through environment variables:

* `ROSWWW_INDEX` – path for index page, otherwise packages list would be generated.
* `ROSWWW_DEFAULT` – if set then the index page would redirect to this package's page.
