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

from __future__ import print_function

import time

import rospy

from nist_gear.msg import Order
from nist_gear.msg import VacuumGripperState
from nist_gear.srv import AGVControl
from nist_gear.srv import ConveyorBeltControl
from nist_gear.srv import DroneControl
from nist_gear.srv import SubmitShipment
from nist_gear.srv import VacuumGripperControl
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from math import pi, sqrt
import moveit_commander as mc

import sys


#TODO: submit agvs to deliver orders/kits
#TODO: listen and react to Faulty Products on agvs
#TODO: parse orders
#TODO: parse sensors data
#TODO: locate parts, plan movements
#TODO: number and places of sensors
#TODO

#TODO: control conveyor? (only for development)

def start_competition():
    rospy.loginfo("start_competition() method.")
    rospy.loginfo("Waiting for competition to be ready...")
    rospy.wait_for_service('/ariac/start_competition')
    rospy.loginfo("Competition is now ready.")
    rospy.loginfo("Requesting competition start...")

    try:
        start = rospy.ServiceProxy('/ariac/start_competition', Trigger)
        response = start()
    except rospy.ServiceException as exc:
        rospy.logerr("Failed to start the competition: %s" % exc)
        return False
    if not response.success:
        rospy.logerr("Failed to start the competition: %s" % response)
    else:
        rospy.loginfo("Competition started!")
    return response.success

def setup_competition(comp_class):

    connect_callbacks(comp_class)

    if not comp_class.arm_1_has_been_zeroed:
        comp_class.send_arm_to_state([0.0, -pi/4, pi/2, -pi/4, pi/2, 0], comp_class.arm_1_joint_names, comp_class.arm_1_joint_trajectory_publisher)
        comp_class.arm_1_has_been_zeroed = True

    if not comp_class.arm_2_has_been_zeroed:
        comp_class.send_arm_to_state([pi, -pi/4, pi/2, -pi/4, pi/2, 0], comp_class.arm_2_joint_names, comp_class.arm_2_joint_trajectory_publisher)
        comp_class.arm_2_has_been_zeroed = True

    # move gantry to zero
    comp_class.send_arm_to_state([0, 0, 0], comp_class.gantry_joint_names, comp_class.gantry_joint_trajectory_publisher)

    # reset Grippers
    comp_class.control_gripper(False, "left")
    comp_class.control_gripper(False, "right")

def end_competition():
    rospy.wait_for_service('/ariac/end_competition')
    rospy.ServiceProxy('/ariac/end_competition', Trigger)()


def run_competition():
    group_names = ['Full_Robot', 'Left_Arm', 'Right_Arm', 'Gantry']

    rate = rospy.Rate(1000) # big amount on purpose

    start = time.time()

    while not rospy.is_shutdown():
        rate.sleep()

        time_elapsed = time.time() - start

        if time_elapsed < 12:
            rospy.loginfo(str(time_elapsed))
        else:
            break;

    # moveit_runner = MoveitRunner(group_names, ns='/ariac/gantry')
    #
    # order = get_order()
    # agv_states = {'agv1': [], 'agv2': []}
    #
    # all_known_parts = get_parts_from_cameras()

    # for shipment in order.shipments:
    #     active_agv = 'agv1' if shipment.agv_id == 'agv1' else 'agv2'
    #     agv_state = agv_states[active_agv]
    #
    #     while True:
    #
    #         valid_products = []
    #         for product in shipment.products:
    #             if product not in agv_state:
    #                 valid_products.append(product)
    #
    #         candidate_moves = []
    #         for part in all_known_parts:
    #             for product in valid_products:
    #                 if part.type == product.type:
    #                     candidate_moves.append((part, product))
    #
    #         if candidate_moves:
    #             part, target = candidate_moves[0]
    #
    #             world_target = get_target_world_pose(target, active_agv)
    #             part_location = get_part_type_location(part)
    #
    #             move_successful = moveit_runner.move_part(
    #                 part,
    #                 world_target,
    #                 part_location,
    #                 active_agv
    #             )
    #             if move_successful:
    #                 all_known_parts.remove(part)
    #                 agv_state.append(target)
    #         else:
    #             break
    #
    #     submit_shipment(active_agv, shipment.shipment_type)
    #     agv_states[active_agv] = []

class MyCompetitionClass:
    def __init__(self):
        self.gantry_joint_trajectory_publisher = rospy.Publisher("/ariac/gantry/gantry_controller/command", JointTrajectory, queue_size=10)
        self.arm_1_joint_trajectory_publisher = rospy.Publisher("/ariac/gantry/left_arm_controller/command", JointTrajectory, queue_size=10)
        self.arm_2_joint_trajectory_publisher = rospy.Publisher("/ariac/gantry/right_arm_controller/command", JointTrajectory, queue_size=10)

        self.current_comp_state = None
        self.received_orders = []
        self.gantry_current_joint_state = None
        self.arm_1_current_joint_state = None
        self.arm_2_current_joint_state = None
        self.arm_1_current_gripper_state = None
        self.arm_2_current_gripper_state = None
        self.last_gantry_joint_state_print = time.time()
        self.last_arm_1_joint_state_print = time.time()
        self.last_arm_2_joint_state_print = time.time()
        self.last_arm_1_gripper_state_print = time.time()
        self.last_arm_2_gripper_state_print = time.time()
        self.arm_1_has_been_zeroed = False
        self.arm_2_has_been_zeroed = False
        self.gantry_joint_names = [
            'small_long_joint',
            'torso_rail_joint',
            'torso_base_main_joint',
        ]
        #left
        self.arm_1_joint_names = [
            'left_shoulder_pan_joint',
            'left_shoulder_lift_joint',
            'left_elbow_joint',
            'left_wrist_1_joint',
            'left_wrist_2_joint',
            'left_wrist_3_joint',
        ]
        #right
        self.arm_2_joint_names = [
            'right_shoulder_pan_joint',
            'right_shoulder_lift_joint',
            'right_elbow_joint',
            'right_wrist_1_joint',
            'right_wrist_2_joint',
            'right_wrist_3_joint',
        ]

    def comp_state_callback(self, msg):
        if self.current_comp_state != msg.data:
            rospy.loginfo("Competition state changed: " + str(msg.data))
        self.current_comp_state = msg.data

    def order_callback(self, msg):
        rospy.loginfo("order_callback")
        rospy.loginfo("Received order:\n" + str(msg))
        self.received_orders.append(msg)

    def gantry_joint_state_callback(self, msg):
        if time.time() - self.last_gantry_joint_state_print >= 10:
            rospy.loginfo("Current gantry Joint States (throttled to 0.1 Hz):\n" + str(msg))
            self.last_gantry_joint_state_print = time.time()
        self.gantry_current_joint_state = msg

    def arm_1_joint_state_callback(self, msg):
        if time.time() - self.last_arm_1_joint_state_print >= 10:
            rospy.loginfo("Current Arm 1 Joint States (throttled to 0.1 Hz):\n" + str(msg))
            self.last_arm_1_joint_state_print = time.time()
        self.arm_1_current_joint_state = msg

    def arm_2_joint_state_callback(self, msg):
        if time.time() - self.last_arm_2_joint_state_print >= 10:
            rospy.loginfo("Current Arm 2 Joint States (throttled to 0.1 Hz):\n" + str(msg))
            self.last_arm_2_joint_state_print = time.time()
        self.arm_2_current_joint_state = msg

    def arm_1_gripper_state_callback(self, msg):
        if time.time() - self.last_arm_1_gripper_state_print >= 10:
            rospy.loginfo("Current Arm 1 gripper state (throttled to 0.1 Hz):\n" + str(msg))
            self.last_arm_1_gripper_state_print = time.time()
        self.arm_1_current_gripper_state = msg

    def arm_2_gripper_state_callback(self, msg):
        if time.time() - self.last_arm_2_gripper_state_print >= 10:
            rospy.loginfo("Current Arm 2 gripper state (throttled to 0.1 Hz):\n" + str(msg))
            self.last_arm_2_gripper_state_print = time.time()
        self.arm_2_current_gripper_state = msg

    def send_arm_to_state(self, positions, joints_names, publisher):
        rospy.loginfo("send_arm_to_state method():\npositions" + str(positions))

        msg = JointTrajectory()
        msg.joint_names = joints_names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rospy.Duration(1.0)
        msg.points = [point]
        #TODO: rospy.loginfo("Sending command:\n" + str(msg))
        publisher.publish(msg)

    def send_arm1_to_state(self, positions):
        return self.send_arm_to_state(positions, self.arm_1_joint_names, self.arm_1_joint_trajectory_publisher)

    def send_arm2_to_state(self, positions):
        return self.send_arm_to_state(positions, self.arm_2_joint_names, self.arm_2_joint_trajectory_publisher)

    def send_gantry_to_state(self, positions):
        return self.send_arm_to_state(positions, self.gantry_joint_names, self.gantry_joint_trajectory_publisher)

    def control_gripper(self, enabled, arm):
        if arm not in ("left", "right"):
            raise ValueError('Only two arms ("left", "right")')

        rospy.loginfo("Waiting for gripper control to be ready...")

        service_name = '/ariac/gantry/{}_arm/gripper/control'.format(arm)
        rospy.wait_for_service(service_name)
        rospy.loginfo("Requesting gripper control...")

        try:
            gripper_control = rospy.ServiceProxy(service_name, VacuumGripperControl)
            response = gripper_control(enabled)
        except rospy.ServiceException as exc:
            rospy.logerr("Failed to control the gripper: %s" % exc)
            return False
        if not response.success:
            rospy.logerr("Failed to control the gripper: %s" % response)
        else:
            rospy.loginfo("Gripper controlled successfully")
        return response.success


class MoveitRunner():
    def __init__(self, group_names, node_name='ariac_moveit_example', ns='', robot_description='robot_description'):

        mc.roscpp_initialize(sys.argv)
        #rospy.init_node(node_name, anonymous=True)

        self.robot = mc.RobotCommander(ns+'/'+robot_description, ns)
        self.scene = mc.PlanningSceneInterface(ns)
        self.groups = {}
        for group_name in group_names:
            group = mc.MoveGroupCommander(
                group_name,
                robot_description=ns+'/'+robot_description,
                ns=ns
            )
            group.set_goal_tolerance(0.05)
            self.groups[group_name] = group

        self.define_preset_locations()
        self.goto_preset_location('start')

    def define_preset_locations(self):
        locations = {}

        name = 'start'
        gantry = [0, 0, 0]
        left = [0.0, -pi/4, pi/2, -pi/4, pi/2, 0]
        right = [pi, -pi/4, pi/2, -pi/4, pi/2, 0]
        locations[name] = (gantry, left, right)

        name = 'bin3'
        gantry = [4.0, -1.1, 0.]
        left = [0.0, 0.0, pi/4, 0, pi/2, 0]
        right = None
        locations[name] = (gantry, left, right)

        name = 'bin4'
        gantry = [5.0, -1.2, 0.]
        left = [0.0, 0.0, pi/4, 0, pi/2, 0]
        right = None
        locations[name] = (gantry, left, right)

        name = 'shelf1'
        gantry = [2.5, -2.3, 0.]
        left = [-pi/2, -pi, -pi/2, -pi/2, 0, 0]
        right = None
        locations[name] = (gantry, left, right)

        name = 'standby'
        gantry = None
        left = [0.0, -pi/4, pi/2, -pi/4, pi/2, 0]
        right = [pi, -pi/4, pi/2, -pi/4, pi/2, 0]
        locations[name] = (gantry, left, right)

        name = 'agv1'
        gantry = [-0.6, -6.9, 0]
        left = [0.0, 0.0, pi/4, 0, pi/2, 0]
        right = None
        locations[name] = (gantry, left, right)

        name = 'agv2'
        gantry = [0.6, 6.9, pi]
        left = [0.0, 0.0, pi/4, 0, pi/2, 0]
        right = None
        locations[name] = (gantry, left, right)

        self.locations = locations

    def goto_preset_location(self, location_name):
        group = self.groups['Full_Robot']
        gantry, left, right = self.locations[location_name]
        location_pose = group.get_current_joint_values()

        if gantry:
            location_pose[:3] = gantry
        if left:
            location_pose[3:-6] = left
        if right:
            location_pose[-6:] = right

        # If the robot controller reports a path tolerance violation,
        # this will automatically re-attempt the motion
        MAX_ATTEMPTS = 5
        attempts = 0
        while not group.go(location_pose, wait=True):
            attempts += 1
            assert(attempts < MAX_ATTEMPTS)

    def move_part(self, part, target, part_location, agv):
        # This example only uses the left arm
        group = self.groups['Left_Arm']
        gm = GripperManager(ns='/ariac/gantry/left_arm/gripper/')

        near_pick_pose = copy.deepcopy(part.pose)
        pick_pose = copy.deepcopy(part.pose)
        place_pose = copy.deepcopy(target.pose)

        near_pick_pose.position.z += 0.1
        pick_pose.position.z += 0.015
        place_pose.position.z += 0.1

        self.goto_preset_location(part_location)
        gm.activate_gripper()

        path = [near_pick_pose, pick_pose]
        self.cartesian_move(group, path)

        num_attempts = 0
        MAX_ATTEMPTS = 20
        while not gm.is_object_attached() and num_attempts < MAX_ATTEMPTS:
            num_attempts += 1
            rospy.sleep(0.1)

        if not gm.is_object_attached():
            self.goto_preset_location(part_location)
            self.goto_preset_location('standby')
            self.goto_preset_location('start')
            return False

        if 'shelf' in part_location:
            self.goto_preset_location(part_location)
        self.goto_preset_location('standby')
        self.goto_preset_location('start')
        self.goto_preset_location(agv)

        path = [place_pose]
        self.cartesian_move(group, path)

        gm.deactivate_gripper()

        self.goto_preset_location('standby')
        self.goto_preset_location('start')
        return True

    def cartesian_move(self, group, waypoints):
        (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0.0)
        group.execute(plan, wait=True)


class GripperManager():
    def __init__(self, ns):
        self.ns = ns

    def activate_gripper(self):
        rospy.wait_for_service(self.ns + 'control')
        rospy.ServiceProxy(self.ns + 'control', VacuumGripperControl)(True)

    def deactivate_gripper(self):
        rospy.wait_for_service(self.ns + 'control')
        rospy.ServiceProxy(self.ns + 'control', VacuumGripperControl)(False)

    def is_object_attached(self):
        status = rospy.wait_for_message(self.ns + 'state', VacuumGripperState)
        return status.attached



def submit_shipment(agv, name):
    rospy.wait_for_service('/ariac/' + agv)
    rospy.ServiceProxy('/ariac/' + agv, AGVControl)(name)


def get_order():
    order = rospy.wait_for_message('/ariac/orders', Order)
    return order


def get_part_type_location(part):
    # rospy.wait_for_service('/ariac/material_locations')
    # response = rospy.ServiceProxy('/ariac/material_locations',
    #                               GetMaterialLocations)(part.type)
    # reachable_location = None
    # for loc in response.storage_units:
    #     if 'shelf' in loc.unit_id or 'bin' in loc.unit_id:
    #         reachable_location = loc.unit_id
    #         break
    if part.type == 'piston_rod_part_blue':
        reachable_location = 'bin4'
    elif part.type == 'gear_part_green':
        reachable_location = 'bin3'

    assert(reachable_location), "This implementation only reaches shelves/bins"
    return reachable_location

def get_parts_from_cameras():
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # wait for all cameras to be broadcasting
    all_topics = rospy.get_published_topics()
    camera_topics = [t for t, _ in all_topics if '/ariac/logical_camera' in t]
    for topic in camera_topics:
        rospy.wait_for_message(topic, LogicalCameraImage)

    camera_frame_format = r"logical_camera_[0-9]+_(\w+)_[0-9]+_frame"
    all_frames = yaml.safe_load(tf_buffer.all_frames_as_yaml()).keys()
    part_frames = [f for f in all_frames if re.match(camera_frame_format, f)]

    objects = []
    for frame in part_frames:
        try:
            world_tf = tf_buffer.lookup_transform(
                'world',
                frame,
                rospy.Time(),
                rospy.Duration(0.1)
            )
            ee_tf = tf_buffer.lookup_transform(
                frame,
                'left_ee_link',
                rospy.Time(),
                rospy.Duration(0.1)
            )
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
            continue

        # remove stale transforms
        tf_time = rospy.Time(
            world_tf.header.stamp.secs,
            world_tf.header.stamp.nsecs
        )
        if rospy.Time.now() - tf_time > rospy.Duration(1.0):
            continue

        model = Model()
        model.type = re.match(camera_frame_format, frame).group(1)
        model.pose.position = world_tf.transform.translation
        model.pose.orientation = ee_tf.transform.rotation
        objects.append(model)
    return objects


def get_target_world_pose(target, agv):
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

    tf_msg = TransformStamped()
    tf_msg.header.frame_id = 'kit_tray_1' if agv == 'agv1' else 'kit_tray_2'
    tf_msg.header.stamp = rospy.Time()
    tf_msg.child_frame_id = 'target_frame'
    tf_msg.transform.translation = target.pose.position
    tf_msg.transform.rotation = target.pose.orientation

    for _ in range(5):
        tf_broadcaster.sendTransform(tf_msg)

    # tf lookup fails occasionally, this automatically retries the lookup
    MAX_ATTEMPTS = 10
    attempts = 0
    while attempts < MAX_ATTEMPTS:
        try:
            world_target_tf = tf_buffer.lookup_transform(
                'world',
                'target_frame',
                rospy.Time(),
                rospy.Duration(0.1)
            )
            ee_target_tf = tf_buffer.lookup_transform(
                'target_frame',
                'left_ee_link',
                rospy.Time(),
                rospy.Duration(0.1)
            )
            break
        except:
            continue

    world_target = copy.deepcopy(target)
    world_target.pose.position = world_target_tf.transform.translation
    world_target.pose.orientation = ee_target_tf.transform.rotation
    return world_target

def connect_callbacks(comp_class):
    rospy.loginfo("connect_callbacks")

    comp_state_sub = rospy.Subscriber("ariac/competition_state", String, comp_class.comp_state_callback)
    order_sub = rospy.Subscriber("/ariac/orders", Order, comp_class.order_callback)
    gantry_state_sub = rospy.Subscriber("/ariac/gantry/gantry_controller/state", JointTrajectoryControllerState, comp_class.gantry_joint_state_callback)
    arm1_joint_state_sub = rospy.Subscriber("/ariac/gantry/left_arm_controller/state", JointTrajectoryControllerState, comp_class.arm_1_joint_state_callback)
    arm2_joint_state_sub = rospy.Subscriber("/ariac/gantry/right_arm_controller/state", JointTrajectoryControllerState, comp_class.arm_2_joint_state_callback)
    gripper1_state_sub = rospy.Subscriber("/ariac/gantry/left_arm/gripper/state", VacuumGripperState, comp_class.arm_1_gripper_state_callback)
    gripper2_state_sub = rospy.Subscriber("/ariac/gantry/right_arm/gripper/state", VacuumGripperState, comp_class.arm_2_gripper_state_callback)


def control_agv(shipment_type, agv_num):
    if agv_num not in (1,2):
        raise ValueError('agv_num must be 1 or 2')

    rospy.loginfo("Waiting for agv{} control to be ready...".format(agv_num))
    name = '/ariac/agv{}'.format(agv_num)
    rospy.wait_for_service(name)
    rospy.loginfo("agv{} control is now ready.".format(agv_num))
    rospy.loginfo("Requesting agv control...")

    try:
        agv_control = rospy.ServiceProxy(name, AGVControl)
        response = agv_control(shipment_type)
    except rospy.ServiceException as exc:
        rospy.logerr("Failed to control the agv: %s" % exc)
        return False
    if not response.success:
        rospy.logerr("Failed to control the agv: %s" % response)
    else:
        rospy.loginfo("agv controlled successfully")
    return response.success


def submit_shipment(shipment_type, agv_num):
    if agv_num not in (1,2):
        raise ValueError('agv_num must be 1 or 2')

    rospy.loginfo("Waiting for submit shipment to be ready...".format(agv_num))
    name = '/ariac/submit_shipment'
    rospy.wait_for_service(name)
    rospy.loginfo("submit_shipment is now ready.")
    rospy.loginfo("Requesting shipment")

    try:
        submit_shipment = rospy.ServiceProxy(name, SubmitShipment)
        response = submit_shipment(destination_id=str(agv_num), shipment_type=shipment_type)
    except rospy.ServiceException as exc:
        rospy.logerr("Failed to submit shipment: %s" % exc)
        return False
    if not response.success:
        rospy.logerr("Failed to submit shipment: %s" % response)
    else:
        rospy.loginfo("shipment submitted successfully")
    return response.success
