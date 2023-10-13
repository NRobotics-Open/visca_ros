#!/usr/bin/env bash


# Add autostart on bootup

# Absolute path to this script, e.g. /home/user/bin/foo.sh
SCRIPT=$(readlink -f "$0")
# Absolute path this script is in, thus /home/user/bin
SCRIPT_DIR=$(dirname "$SCRIPT")

mkdir -p /home/$USER/.config/autostart

cat <<EOF >/home/$USER/.config/autostart/CameraLauncher.desktop
[Desktop Entry]
Type=Application
Name=VISCA Camera Launcher
Exec=bash $SCRIPT_DIR/start.sh &
Comment=VISCA Camera Autostart
EOF