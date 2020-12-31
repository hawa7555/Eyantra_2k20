#! /usr/bin/env python

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math


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


        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def set_joint_angles(self, arg_list_joint_angles):
        # print("Harshal")
        # print(self._group_names)
            

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        for angle in arg_list_joint_angles:
            list_joint_values[ self.dict_joints_index[angle] ] = arg_list_joint_angles[angle]
        
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

    def set_shoulder(self, arg_list_joint_angles):
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

    # lst_joint_angles_1 = [math.radians(0),
    #                       math.radians(0),
    #                       math.radians(0),
    #                       math.radians(0),
    #                       math.radians(0),
    #                       math.radians(0)]

    # lst_joint_angles_2 = [math.radians(133),
    #                       math.radians(-59),
    #                       math.radians(13),
    #                       math.radians(-134),
    #                       math.radians(47),
    #                       math.radians(23)]

    # lst_joint_angles_3 = [math.radians(-70),
    #                       math.radians(-54),
    #                       math.radians(-139),
    #                       math.radians(-174),
    #                       math.radians(9),
    #                       math.radians(6)]

    dict_angles_pick = { "elbow_joint": math.radians(120), "shoulder_lift_joint": math.radians(-90) ,
    "shoulder_pan_joint": math.radians(0) , "wrist_1_joint": math.radians(20),
    "wrist_2_joint": math.radians(90), "wrist_3_joint": math.radians(0) }

    dict_angles_1 = { "elbow_joint": math.radians(0), "shoulder_lift_joint": math.radians(0) ,
    "shoulder_pan_joint": math.radians(0) , "wrist_1_joint": math.radians(0),
    "wrist_2_joint": math.radians(0), "wrist_3_joint": math.radians(0) }

    dict_angles_pick_adjust = { "shoulder_pan_joint" :  math.radians(-10), "shoulder_lift_joint": -0.9,
    "wrist_1_joint": -1.60, "elbow_joint": 2 }

    dict_angles_pick_semi_final = { "elbow_joint": 1.648, "shoulder_lift_joint": -1.28 ,
    "shoulder_pan_joint": -4.06 , "wrist_1_joint": -1.56,
    "wrist_2_joint": 4.755, "wrist_3_joint": 0.00 }


    dict_angles_pick_final_1_1 = { "shoulder_pan_joint": 3.24002734077,
    "shoulder_lift_joint": -2.08165957908,
    "elbow_joint": -1.23676623066, "wrist_1_joint": 1.96097401511,
    "wrist_2_joint": 4.57602036975, "wrist_3_joint": -3.76778258036 }

    dict_angles_pick_final_1_2 = { "shoulder_pan_joint": 5.95281523125,
    "shoulder_lift_joint": 5.04994938364,
    "elbow_joint": 2.05531894658, "wrist_1_joint": -2.53122378859,
    "wrist_2_joint": -1.35655440932, "wrist_3_joint": -1.05514087658 }


    dict_angles_pick_final_2 = { "shoulder_pan_joint": 3.80854369904,
    "shoulder_lift_joint": -1.80594042297,
    "elbow_joint": -2.02144348629, "wrist_1_joint": 5.42100078136,
    "wrist_2_joint": 1.59267913833, "wrist_3_joint": -2.91815692631 }

    # dict_angles_3 = { "elbow_joint": math.radians(-70), "shoulder_lift_joint": math.radians(-54) ,
    # "shoulder_pan_joint": math.radians(-139) , "wrist_1_joint": math.radians(-174),
    # "wrist_2_joint": math.radians(9), "wrist_3_joint": math.radians(6) }
    
    object_1 = {'x': 0.56, 'y': 0.0, 'z': 0.61, 'r': 0.000002, 'p': 0.000001, 'y': -1.349999}
    object_2 = {'x': 0.46, 'y': 0.22, 'z': 0.612498, 'r': 0.000024, 'p': 0.000008, 'y': 0.0}
    object_3 = {'x': 0.54, 'y': -0.24, 'z': 0.609995, 'r': 0.000002, 'p': -0.000004, 'y': -0.789999}

    dict_angles_pick_shoulder = {"shoulder_lift_joint" : -0.2, "wrist_1_joint": -0.1 }
    
    while not rospy.is_shutdown():
        # ur5.handAngle(20)
        # rospy.sleep(2)
        # ur5.set_joint_angles(dict_angles_pick)
        # rospy.sleep(2)
        # ur5.set_joint_angles(dict_angles_pick_final)
        # rospy.sleep(2)
        ur5.set_joint_angles(dict_angles_pick_final_1_1)
        rospy.sleep(2)
        ur5.set_joint_angles(dict_angles_pick_final_1_2)
        rospy.sleep(20)
        # ur5.set_shoulder(dict_angles_pick_shoulder)
        # rospy.sleep(30)
        # ur5.set_joint_angles(dict_angles_pick_adjust)
        # rospy.sleep(2)
        # ur5.openGripper()
        # rospy.sleep(2)
        # ur5.closeGripper()
        # rospy.sleep(2)
        # ur5.set_joint_angles(dict_angles_3)
        # rospy.sleep(2)
        # ur5.set_joint_angles(dict_angles_pick_adjust)
        # rospy.sleep(2)

    del ur5


if __name__ == '__main__':
    main()
