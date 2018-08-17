./change_boot_part.sh
./init_image.sh qemu_build_17082018 raspbian_nov_2017
./software_install.sh
./network_setup.sh
./ros_install.sh https://github.com/CopterExpress/clever.git master True
