# Building a modified image

Sometimes it is necessary to build an image with copter settings different from the release version of the image. There are several ways to do this.

## Building preparation

Install [docker](https://www.docker.com):

```bash
curl -fsSL https://get.docker.com -o get-docker.sh
sh get-docker.sh
```

## Local build with modified Clover settings

* Place the Clover configuration folders (`launch`, `map` and `camera_info`) in the `builder/clever-config` [folder](../../builder/clever-config) in the clever-show source directory.
  * All files from the `launch` folder will be copied to the `/home/pi/catkin_ws/src/clever/clever/launch` directory in the built image.
  * All files from the `map` folder will be copied to the `/home/pi/catkin_ws/src/clever/aruco_pose/map` directory in the built image.
  * All files from the `camera_info` folder will be copied to the `/home/pi/catkin_ws/src/clever/clever/camera_info` directory in the built image.
* Build your image with docker:

```bash
cd <source-dir>
sudo docker run --privileged -it --rm -v /dev:/dev -v $(pwd):/mnt goldarte/img-tool:v0.5
```

## Manual image customization

* Extract the file with the downloaded image, navigate to the directory with this image, and enter the image collector console using the command:

```bash
cd <image-dir
sudo docker run --privileged -it --rm -v /dev:/dev -v $(pwd):/mnt goldarte/img-tool:v0.5 img-chroot /mnt/<IMAGE>
```

where `<IMAGE>` is the name of the image file. In the opened terminal you can customize the image using standard programs (nano, git, cp, apt-get).

* You can transfer external files to an image using the command:

```bash
sudo docker run --privileged -it --rm -v /dev:/dev -v $(pwd):/mnt goldarte/img-tool:v0.5 img-chroot /mnt/<IMAGE> copy /mnt/<MOVE_FILE> <MOVE_TO>
```

where `<MOVE_FILE>` is the file to be moved to the image (location relative to the folder with the image, for example `.../builder/assets/clever-show.service`) and `<MOVE_TO>` is the path in the image where the file should be moved.

* If the image does not have enough space for all the necessary files, you can extend the image with the command:

```bash
sudo docker run --privileged -it --rm -v /dev:/dev -v $(pwd):/mnt goldarte/img-tool:v0.5 img-resize /mnt/<IMAGE> max <SIZE>
```

where `<SIZE>` is the size in bytes. For example, 5G would mean 5GB and 5M would mean 5MB.

* After extending the image, you can compress it to the minimum size + 10MB with the command

```bash
sudo docker run --privileged -it --rm -v /dev:/dev -v $(pwd):/mnt goldarte/img-tool:v0.5 img-resize /mnt/<IMAGE> min
```

## Builder scripts modification

The article on changing the image building and custom building scripts is avialiable [here](https://clever.copterexpress.com/ru/image_building.html).
