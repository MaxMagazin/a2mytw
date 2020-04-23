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
    rospy.init_node("ariac_my_team_winner_node")
    rospy.loginfo("Initting ariac_my_team_winner_node python")

    comp_class = ariac_my_team_winner.MyCompetitionClass()
    ariac_my_team_winner.connect_callbacks(comp_class)

    rospy.loginfo("Setup complete, start competition.")
    time.sleep(1)
    ariac_my_team_winner.start_competition()

    if not comp_class.arm_1_has_been_zeroed:
        comp_class.send_arm_to_state([-1.57, 0, 0, 0, 0, 0], comp_class.arm_1_joint_names, comp_class.arm_1_joint_trajectory_publisher)
        comp_class.arm_1_has_been_zeroed = True

    if not comp_class.arm_2_has_been_zeroed:
        comp_class.send_arm_to_state([1.57, 0, 0, 0, 0, 0], comp_class.arm_2_joint_names, comp_class.arm_2_joint_trajectory_publisher)
        comp_class.arm_2_has_been_zeroed = True

    # move gantry to zero
    comp_class.send_arm_to_state([0, 0, 0], comp_class.gantry_joint_names, comp_class.gantry_joint_trajectory_publisher)

    # reset Grippers
    comp_class.control_gripper(False, "left")
    comp_class.control_gripper(False, "right")

    rospy.spin()


if __name__ == '__main__':
    main()
