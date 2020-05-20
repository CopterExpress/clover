# roswww_static

roswww_static creates a static web directory for your ROS-powered system with symlinks to all the `www` subfolders found in your ROS packages. This way you can use any external web server (e. g. [nginx](https://nginx.org/), [Monkey](https://github.com/monkey/monkey), [Caddy](https://caddyserver.com)) to serve you static data, in compatible with roswww manner.

Note: you should configure your web server to make it follow symlinks.

## Instructions

* Run `main.py` node and it will generate the symlinks and index file.
* Point your static web server path to `<roswww_static_path>/www`.

You can rerun roswww_static if the list of installed packages changes.

## Parameters

* `index` – path for index page, otherwise packages list would be generated.
