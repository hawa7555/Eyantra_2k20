#! /usr/bin/env python

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math

from tf.transformations import quaternion_from_euler

class Ur5Moveit:

    dict_joints_index = { "shoulder_pan_joint": 0, "shoulder_lift_joint": 1,
    "elbow_joint": 2, "wrist_1_joint": 3, "wrist_2_joint": 4, "wrist_3_joint": 5 }

    # Constructor
    def __init__(self):

        rospy.init_node('node_eg3_set_joint_angles', anonymous=True)

        self._planning_group = "arm_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def go_to_pose(self, arg_pose):
        print("Harshal")
        print(arg_pose)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move
        self._group.stop()
        self._group.clear_pose_targets()

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    def set_joint_angles(self, arg_list_joint_angles):
        
        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        for angle in arg_list_joint_angles:
            list_joint_values[ self.dict_joints_index[angle] ] = arg_list_joint_angles[angle]

        rospy.loginfo('\033[94m' + ">>> New Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)
        self._group.set_joint_value_target(list_joint_values)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    
    def openGripper(self):
        hand_group = moveit_commander.MoveGroupCommander("hand_group")
        hand_group.set_named_target("openHand")
        flag_plan = hand_group.go(wait=True)
        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> Hand Open Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> Hand Open fail" + '\033[0m')

    
    def closeGripper(self):
        hand_group = moveit_commander.MoveGroupCommander("hand_group")
        hand_group.set_named_target("closedHand")
        flag_plan = hand_group.go(wait=True)
        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> Hand Close Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> Hand Close fail" + '\033[0m')

    # def removeCollission(self):
    #     grasping_group = 'hand_group'
    #     touch_links = self._robot.get_link_names(group=grasping_group)
    #     scene.attach_box(eef_link, box_name, touch_links=touch_links)

    def go_to_predefined_pose(self, arg_pose_name):
        rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group.set_named_target(arg_pose_name)
        plan = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')

    def handAngle(self, angleValue):
        hand_group = moveit_commander.MoveGroupCommander("hand_group")
        list_joint_values = hand_group.get_current_joint_values()
        list_joint_values[0] = math.radians(angleValue)
        #  print("harshal007")
        #  print(list_joint_values)
        hand_group.set_joint_value_target(list_joint_values)
        hand_group.plan()
        flag_plan = hand_group.go(wait=True)
        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> Hand angle success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> Hand angle failed" + '\033[0m')

        return flag_plan

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    ur5 = Ur5Moveit()

    ur5_pose_1 = geometry_msgs.msg.Pose()
    ur5_pose_1.position.x = 0.509817071161
    ur5_pose_1.position.y = -0.0593452799362
    # ur5_pose_1.position.z = 0.990867076642
    ur5_pose_1.position.z = 1.070867076642

    # ur5_pose_1.orientation.x = -0.999999995957
    # ur5_pose_1.orientation.y = 4.37354574363e-05
    # ur5_pose_1.orientation.z = 7.85715579538e-05
    # ur5_pose_1.orientation.w = 2.12177767514e-09

    quaternion = quaternion_from_euler(-1.31769539542, -0.0170005377369, -0.833568602861)

    ur5_pose_1.orientation.x = quaternion[0]
    ur5_pose_1.orientation.y = quaternion[1]
    ur5_pose_1.orientation.z = quaternion[2]
    ur5_pose_1.orientation.w = quaternion[3]

    ur5_pose_2 = geometry_msgs.msg.Pose()
    ur5_pose_2.position.x = 0.00
    ur5_pose_2.position.y = 0.710000
    ur5_pose_2.position.z = 1.170867076642

    quaternion = quaternion_from_euler(-1.31769539542, -0.0170005377369, -0.833568602861)

    ur5_pose_2.orientation.x = quaternion[0]
    ur5_pose_2.orientation.y = quaternion[1]
    ur5_pose_2.orientation.z = quaternion[2]
    ur5_pose_2.orientation.w = quaternion[3]


    ur5_pose_3 = geometry_msgs.msg.Pose()
    ur5_pose_3.position.x = 0.465352890709
    ur5_pose_3.position.y = 0.227455234033
    ur5_pose_3.position.z = 0.996101367752
    # ur5_pose_3.position.z = 0.926101367752
    
    quaternion = quaternion_from_euler(-1.52968925634, -0.0130863995715, 2.01363577288)

    ur5_pose_3.orientation.x = quaternion[0]
    ur5_pose_3.orientation.y = quaternion[1]
    ur5_pose_3.orientation.z = quaternion[2]
    ur5_pose_3.orientation.w = quaternion[3]



    ur5_pose_4 = geometry_msgs.msg.Pose()
    ur5_pose_4.position.x = 0.00
    ur5_pose_4.position.y = -0.71
    ur5_pose_4.position.z = 1.170867076642
    
    quaternion = quaternion_from_euler(-1.52968925634, -0.0130863995715, 2.01363577288)

    ur5_pose_4.orientation.x = quaternion[0]
    ur5_pose_4.orientation.y = quaternion[1]
    ur5_pose_4.orientation.z = quaternion[2]
    ur5_pose_4.orientation.w = quaternion[3]

    # ur5_object_1 = geometry_msgs.msg.Pose()
    # ur5_object_1.position.x = 0.56
    # ur5_object_1.position.y = 0.0
    # ur5_object_1.position.z = 0.61

    # quaternion = quaternion_from_euler(0.000002, 0.000001, -1.349999)

    # ur5_object_1.orientation.x = quaternion[0]
    # ur5_object_1.orientation.y = quaternion[1]
    # ur5_object_1.orientation.z = quaternion[2]
    # ur5_object_1.orientation.w = quaternion[3]

    # ur5_object_1.orientation.x = -0.999999995957
    # ur5_object_1.orientation.y = 4.37354574363e-05
    # ur5_object_1.orientation.z = 7.85715579538e-05
    # ur5_object_1.orientation.w = 2.12177767514e-09

    # dict_angles_pick_adjust = { "wrist_1_joint": math.radians(-90) }

    ur5.go_to_predefined_pose("straightUp")
    rospy.sleep(2)

    ur5.openGripper()
    rospy.sleep(2)

    while not rospy.is_shutdown():
        # ur5.go_to_pose(ur5_pose_1)
        # rospy.sleep(2)
        # ur5_pose_1.position.z = 0.915867076642
        # ur5.go_to_pose(ur5_pose_1)
        # rospy.sleep(2)
        # ur5.handAngle(9)
        # rospy.sleep(2)
        # ur5_pose_1.position.z = 1.070867076642
        # ur5.go_to_pose(ur5_pose_1)
        # rospy.sleep(2)
        # ur5.go_to_pose(ur5_pose_2)
        # rospy.sleep(2)
        # ur5.openGripper()
        # rospy.sleep(20)


        ur5.go_to_pose(ur5_pose_3)
        rospy.sleep(2)
        ur5_pose_3.position.z = 0.926101367752
        ur5.go_to_pose(ur5_pose_3)
        rospy.sleep(2) 
        ur5.handAngle(9)
        rospy.sleep(2)
        ur5_pose_3.position.z = 1.070867076642
        ur5.go_to_pose(ur5_pose_3)
        rospy.sleep(2)
        ur5.go_to_pose(ur5_pose_4)
        rospy.sleep(20)
        # ur5.openGripper()
        # rospy.sleep(20)

    del ur5


if __name__ == '__main__':
    main()