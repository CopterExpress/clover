# Animation export Blender addon

The Addon for Blender is designed to convert Blender's copters flight animations into flight paths for each copter of the animation, including the color of objects at any given time.

## Installation and configuration

* Download and install the latest version of Blender 2.83 from [the official website](https://www.blender.org/download/).
* Open Blender, select `Edit > Preferences` from the top menu. In the opened settings window, select `Add-ons` in the side panel. Click the button `Install...` in the upper right corner of the window. In the dialog box, open the path to the addon folder [clever-show/blender-addon](.../../blender-addon/) and select the file `addon.py`. Click `Install Add-on from file...`. Addon is now installed.
* After installing the addon, tick the `Import-Export: clever-show animation (.csv)` checkbox to activate the addon.

Addon is now active and ready to go. You will not need to perform these operations at further Blender startups on the same PC.

## Exporting with the addon

* To open the export dialog box, click on the top menu `File > Export > clever-show animation (.csv)`. In the export window that opens, you should select the destination export path and the name of the folder that the addon will create during the export process. The export options panel is available in the side menu:
* `Use name filter for objects` - checkbox determines if the object filter will be used while saving the paths. If this option is disabled, the paths of all visible objects will be exported. `Name identifier` - object name filter. If checkbox `Use name filter for objects` is active, only paths of objects containing this value in the name will be saved.
* `Show detailed animation warnings` - checkbox determines whether the animation's speeds and distances limits warnings will be displayed.
* `Speed limit` - warnings will be displayed if the specified speed limit is violated.
* `Distance limit` - warnings will be displayed if the specified minimum distance between drones is violated.

After configuring the required parameters, press the `Export clever-show animation` button. Animations of the specified objects from the Blender project will be exported to the specified folder in the `.csv` format.

## Deactivation and removal

To deactivate an addon, uncheck the checkbox next to the addon name as described [above] (#installation and configuration).

For more information, click the arrow icon to the left of the activation field. There are also buttons in the unfolded block:

* `Documentation` - leads to the addon's documentation page.
* `Report a bug` - leads to the issues page of the clever-show repository.
* `Remove` - removes the addon (before installing a new version it is recommended to remove the old one).
