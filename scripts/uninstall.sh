#!/usr/bin/env bash


# Remove autostart on bootup

# Absolute path to this script, e.g. /home/user/bin/foo.sh
SCRIPT=$(readlink -f "$0")
# Absolute path this script is in, thus /home/user/bin
SCRIPT_DIR=$(dirname "$SCRIPT")

rm /home/$USER/.config/autostart/CameraLauncher.desktop