# Hostname

[By default](microsd_images.md) hostname of the Clever drone is set to `clever-xxxx`, where `xxxx` are random numbers. Hostname matches [Wi-Fi SSID](wifi.md).

Thus, Clever is accessible on machines, that support mDNS, under a name `clever-xxxx.local`. You can use this name for accessing Clever with SSH:

```bash
ssh pi@clever-xxxx.local
```

Also, this name can be used in place of IP-address to open Clever web pages in browser, etc.

## Changing hostname

In some situations, it is necessary to change Clever's hostname. Use `hostnamectl` utility for that:

```bash
sudo hostnamectl set-hostname newname
```

Where `newname` is the new name of the machine. `hostnamectl` utility will change the name in `/etc/hostname` file.

Also, you have to write down the new name to `/etc/hosts` file:

```txt
127.0.1.1	newname newname.local
```

Setting `newname.local` is necessary for allowing ROS to resolve this name in situations, where all the network interfaces are down (when Wi-Fi is turned off or disconnected).
