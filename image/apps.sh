#!/bin/bash

set -e

##################################################################################################################################
# Image software installation
##################################################################################################################################

# Add apt key to allow local mirror usage during image build
#wget -O - ftp://192.168.0.10/coex-mirror.gpg | apt-key add -
# Generate a backup of the original source.list
#cp /etc/apt/sources.list /var/sources.list.bak
# Add the local mirror as the first priority repository
#wget -O - ftp://192.168.0.10/coex-mirror.list 2>/dev/null | cat - /etc/apt/sources.list > /var/sources.list && mv /var/sources.list /etc/apt/sources.list

echo "\033[0;31m\033[1m$(date) | #1 apt cache update\033[0m\033[0m"

# Clean repostory cache
apt-get clean
# Update repository cache
apt-get update
# && apt upgrade -y

echo "\033[0;31m\033[1m$(date) | #2 Software installation\033[0m\033[0m"

# TODO: Use dnsmasq instead of isc-dhcp-server
apt-get install --no-install-recommends -y \
  unzip \
  zip \
  ipython \
  screen \
  byobu \
  nmap \
  lsof \
  python-pip \
  git \
  isc-dhcp-server \
  tmux \
  vim \
  ipython3 \
  python3-pip

echo "\033[0;31m\033[1m$(date) | #3 Write to /etc/wpa_supplicant/wpa_supplicant.conf\033[0m\033[0m"

# TODO: Use wpa_cli insted direct file edit
echo "
network={
    ssid=\"CLEVER\"
    mode=2
    key_mgmt=WPA-PSK
    psk=\"cleverwifi\"
    frequency=2437
}" >> /etc/wpa_supplicant/wpa_supplicant.conf

echo "\033[0;31m\033[1m$(date) | #4 Write STATIC to /etc/dhcpcd.conf\033[0m\033[0m"

echo "
interface wlan0
static ip_address=192.168.11.1/24" >> /etc/dhcpcd.conf

echo "\033[0;31m\033[1m$(date) | #5 Write iface to /etc/default/isc-dhcp-server\033[0m\033[0m"

# https://www.shellhacks.com/ru/sed-find-replace-string-in-file/
sed -i 's/INTERFACESv4=\"\"/INTERFACESv4=\"wlan0\"/' /etc/default/isc-dhcp-server

echo "\033[0;31m\033[1m$(date) | #6 Write dhcp declaration subnet to /etc/dhcp/dhcpd.conf\033[0m\033[0m"

echo "subnet 192.168.11.0 netmask 255.255.255.0 {
  range 192.168.11.11 192.168.11.254;
  #option domain-name-servers 8.8.8.8;
  #option domain-name "rpi.local";
  option routers 192.168.11.1;
  option broadcast-address 192.168.11.255;
  default-lease-time 600;
  max-lease-time 7200;
}" >> /etc/dhcp/dhcpd.conf

echo "\033[0;31m\033[1m$(date) | #7 Write start script for dhcpd to /etc/network/if-up.d/isc-dhcp-server\033[0m\033[0m"

echo "#!/bin/sh
if [ \"\$IFACE\" = \"--all\" ];
then sleep 10 && systemctl start isc-dhcp-server.service &
fi
" > /etc/network/if-up.d/isc-dhcp-server \
  && chmod +x /etc/network/if-up.d/isc-dhcp-server

echo "\033[0;31m\033[1m$(date) | #8 Write magic script for rename SSID to /etc/rc.local\033[0m\033[0m"

RENAME_SSID="sudo sed -i.OLD \"s/CLEVER/CLEVER-\$(head -c 100 /dev/urandom | xxd -ps -c 100 | sed -e 's/[^0-9]//g' | cut -c 1-4)/g\" /etc/wpa_supplicant/wpa_supplicant.conf && sudo sed -i '/sudo sed/d' /etc/rc.local && sudo reboot"

sed -i "19a$RENAME_SSID" /etc/rc.local

echo "\033[0;31m\033[1m$(date) | #9 End of software installation\033[0m\033[0m"
