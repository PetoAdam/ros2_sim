#!/bin/bash

# Run apt update to fetch package descriptions
sudo apt update -qqy

# Add robotpkg as a source repository
sudo apt install -qqy lsb-release curl
sudo mkdir -p /etc/apt/keyrings
curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
     | sudo tee /etc/apt/keyrings/robotpkg.asc

# Force using 'jammy' repository for Ubuntu 24.04
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub jammy robotpkg" \
     | sudo tee /etc/apt/sources.list.d/robotpkg.list

# Run apt update to fetch package descriptions
sudo apt update -qqy

# Install Pinocchio and its dependencies
sudo apt install -qqy robotpkg-py3*-pinocchio

# Configure environment variables (this image comes with python3.7)
echo 'export PATH=/opt/openrobots/bin:$PATH' >> /home/user/.bashrc
echo 'export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH' >> /home/user/.bashrc
echo 'export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH' >> /home/user/.bashrc
echo 'export PYTHONPATH=/opt/openrobots/lib/python3.7/site-packages:$PYTHONPATH' >> /home/user/.bashrc
echo 'export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH' >> /home/user/.bashrc
