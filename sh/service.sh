#!/bin/bash
cd /lib/systemd/system
sudo cp ~/catkin_ws/src/formation/sh/formation.service .
sudo systemctl daemon-reload
sudo systemctl enable formation.service
