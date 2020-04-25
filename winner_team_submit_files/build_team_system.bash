#!/usr/bin/env bash

YELLOW='\033[0;33m'
GREEN='\033[0;32m'
NOCOLOR='\033[0m'

# Prepare ROS
echo -e "${YELLOW}---Sourcing ROS${NOCOLOR}"
. /opt/ros/melodic/setup.bash
echo -e "${YELLOW}---Sourcing ARIAC${NOCOLOR}"
. /home/ariac-user/ariac_ws/devel/setup.bash


# Install the necessary dependencies for getting the team's source code
# Note: there is no need to use `sudo`.
#echo -e "${YELLOW}---Installing necessary dependencies${NOCOLOR}"
#apt-get update
#apt-get install -y wget unzip


# Create a catkin workspace
echo -e "${YELLOW}---Creating workspace${NOCOLOR}"
mkdir -p ~/my_team_winner_ws/src

if [ -d "/root/my_team_winner_ws" ]; then
  echo -e "${GREEN}OK${NOCOLOR}"
fi

cd ~/my_team_winner_ws/src
echo -e "${YELLOW}---Downloading competition package${NOCOLOR}"
git clone https://github.com/MaxMagazin/a2mytw.git ariac_my_team_winner

echo -e "${YELLOW}---Compiling competition package${NOCOLOR}"
cd ~/my_team_winner_ws
catkin_make
