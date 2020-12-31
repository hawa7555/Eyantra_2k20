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

    def adjustHeight(self, height):
        pose_values = self._group.get_current_pose().pose
        pose_values.position.z = height
        self.go_to_pose(pose_values)

    def adjustJoints(self, arg_list_joint_angles):
        # print("Harshal")
        # print(self._group_names)
            
        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        for angle in arg_list_joint_angles:
            list_joint_values[ self.dict_joints_index[angle] ] += arg_list_joint_angles[angle]
        
        # print("Harshal")
        # print(list_joint_values)

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

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    ur5 = Ur5Moveit()

    dict_angles_pick_final_1_1 = { "shoulder_pan_joint": 3.24002734077,
    "shoulder_lift_joint": -2.08165957908,
    "elbow_joint": -1.23676623066, "wrist_1_joint": 1.96097401511,
    "wrist_2_joint": 4.57602036975, "wrist_3_joint": -3.76778258036 }

    dict_angles_pick_final_1_2 = { "shoulder_pan_joint": -1.5706496436,
    "wrist_1_joint": 1.70972399883 }



    dict_angles_pick_final_2_1 = { "shoulder_pan_joint": 0.242276177758,
    "shoulder_lift_joint": -0.976281651662,
    "elbow_joint": 1.36619805135, "wrist_1_joint": 1.22355829159,
    "wrist_2_joint": -4.71626755086, "wrist_3_joint": -0.199994801097 }

    dict_angles_pick_final_2_2 = { "shoulder_pan_joint": -1.72513027615,
    "shoulder_lift_joint": -1.23117552408, "wrist_1_joint": 0.788750486875 }



    dict_angles_pick_final_3_1 = { "shoulder_pan_joint": -3.37939309135,
    "shoulder_lift_joint": -2.36332709648,
    "elbow_joint": -0.880199038316, "wrist_1_joint": 1.69286421114,
    "wrist_2_joint": 4.72103651885, "wrist_3_joint": -4.9138214124 }

    dict_angles_pick_final_3_2 = { "shoulder_pan_joint": 1.72506024526,
    "shoulder_lift_joint": -2.42091580636, "wrist_1_joint": 1.48283435441 }


    # ur5.go_to_predefined_pose("straightUp")
    # rospy.sleep(2)

    ur5.openGripper()
    rospy.sleep(2)

    while not rospy.is_shutdown():
        ur5.set_joint_angles(dict_angles_pick_final_1_1)
        rospy.sleep(0.5)
        ur5.adjustHeight(0.915867076642)
        # rospy.sleep(1)
        ur5.handAngle(9)
        rospy.sleep(0.5)
        ur5.adjustHeight(1.270867076642)
        rospy.sleep(0.2)
        ur5.set_joint_angles(dict_angles_pick_final_1_2)
        rospy.sleep(0.2)
        ur5.openGripper()
        rospy.sleep(0.5)

        ur5.set_joint_angles(dict_angles_pick_final_2_1)
        rospy.sleep(0.5)
        ur5.adjustHeight(0.926101367752)
        # rospy.sleep(2)
        ur5.handAngle(9)
        rospy.sleep(0.5)
        ur5.adjustHeight(1.070867076642)
        rospy.sleep(0.2)
        ur5.set_joint_angles(dict_angles_pick_final_2_2)
        rospy.sleep(0.2)
        ur5.openGripper()
        rospy.sleep(0.5)

        ur5.set_joint_angles(dict_angles_pick_final_3_1)
        rospy.sleep(0.5)
        ur5.adjustHeight(0.97495443226)
        # rospy.sleep(2)
        ur5.handAngle(10)
        rospy.sleep(0.5)
        ur5.adjustHeight(1.20689469169)
        rospy.sleep(0.2)
        ur5.set_joint_angles(dict_angles_pick_final_3_2)
        rospy.sleep(0.2)
        ur5.openGripper()
        break

    del ur5


if __name__ == '__main__':
    main()