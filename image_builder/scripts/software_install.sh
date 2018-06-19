#!/bin/bash

set -e

##################################################################################################################################
# Image software installation
##################################################################################################################################

echo -e "\033[0;31m\033[1m$(date) | #1 Software installing\033[0m\033[0m"

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
  dnsmasq \
  tmux \
  vim \
  ipython3 \
  python3-pip

echo -e "\033[0;31m\033[1m$(date) | End of software installation\033[0m\033[0m"
