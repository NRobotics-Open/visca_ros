#!/usr/bin/env bash


# Add autostart on bootup
echo "Copying service file ..."
sudo cp visca.service /etc/systemd/system/

echo "Reloading daemon ..."
sudo systemctl daemon-reload

echo "Enabling service ..."
sudo systemctl enable visca.service

echo "Restarting PC!"
sudo reboot