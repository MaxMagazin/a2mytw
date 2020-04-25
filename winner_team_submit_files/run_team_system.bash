#!/usr/bin/env bash

YELLOW='\033[0;33m'
NOCOLOR='\033[0m'
echo -e "${YELLOW}Running team system${NOCOLOR}"
. ~/my_team_winner_ws/devel/setup.bash

# Run the example node
echo -e "${YELLOW}Launching ariac_my_team_winner competitor code${NOCOLOR}"
rosrun ariac_my_team_winner ariac_my_team_winner_node.py
