#!/bin/bash

set -e

##################################################################################################################################
# Image initialisation
##################################################################################################################################

# Add apt key to allow local mirror usage during image build
#wget -O - ftp://192.168.0.10/coex-mirror.gpg | apt-key add -
# Generate a backup of the original source.list
#cp /etc/apt/sources.list /var/sources.list.bak
# Add the local mirror as the first priority repository
#wget -O - ftp://192.168.0.10/coex-mirror.list 2>/dev/null | cat - /etc/apt/sources.list > /var/sources.list && mv /var/sources.list /etc/apt/sources.list

echo -e "\033[0;31m\033[1m$(date) | #1 apt cache update\033[0m\033[0m"

# Clean repostory cache
apt-get clean
# Update repository cache
apt-get update
# && apt upgrade -y

echo -e "\033[0;31m\033[1m$(date) | #2 Write clever information\033[0m\033[0m"

# Clever image version
echo "$1" >> /etc/clever_version
# Origin image file name
echo "${2%.*}" >> /etc/clever_origin

echo -e "\033[0;31m\033[1m$(date) | #3 End initialisation of image\033[0m\033[0m"
