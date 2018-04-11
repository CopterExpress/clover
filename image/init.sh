#!/bin/bash

set -e

##################################################################################################################################
# Image initialisation
##################################################################################################################################

# Clever image version
echo "$1" >> /etc/clever_version
# Origin image file name
echo "${2%.*}" >> /etc/clever_origin

echo "\033[0;31m\033[1m$(date) | #9 End of image initialisation\033[0m\033[0m"
