#!/usr/bin/env python
# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ariac_my_team_winner import ariac_my_team_winner
import rospy
import time

def main():
    rospy.loginfo("Initing ariac_my_team_winner_node python")
    rospy.init_node("ariac_my_team_winner_node")

    comp_class = ariac_my_team_winner.MyCompetitionClass()

    rospy.loginfo("seting up competition.")
    ariac_my_team_winner.setup_competition(comp_class)

    time.sleep(1)

    rospy.loginfo("start competition.")
    ariac_my_team_winner.start_competition()

    # main method
    ariac_my_team_winner.run_competition()

    rospy.loginfo("end competition.")
    ariac_my_team_winner.end_competition()

if __name__ == '__main__':
    main()
