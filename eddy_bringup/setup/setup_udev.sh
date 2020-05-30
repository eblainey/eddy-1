#!/bin/bash
source `rospack find rosbash`/rosbash

# Install the udev rules.
rospd eddy_bringup/udev > /dev/null
echo "Installing udev rules."
sudo cp *.rules /etc/udev/rules.d/
popd > /dev/null
sudo udevadm trigger
