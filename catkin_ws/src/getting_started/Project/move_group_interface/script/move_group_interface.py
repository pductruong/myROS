#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def all_close(goal, actual, tolerance):
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False
    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)
    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal),pose_to_list(actual),tolerance)
    return True

class MoveGroupInterface(object):
    def __init__(self):
        super(MoveGroupInterface, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_interface', anonymous=True)
        robot = moveit_commander.robot.RobotCommander()        
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)        
        planning_frame = move_group.get_planning_frame()
        print("Planning frame: %s" %planning_frame)
        eef_link = move_group.get_end_effector_link()
        print("End effector link: %s" %eef_link)
        group_name = robot.get_group_names()
        print("Availble Planning Groups:", group_name)
        print("Robot state")
        print(robot.get_current_state)
        print(" ")
        self.move_group = move_group
    def go_to_goal(self):
        move_group = self.move_group
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = 0.5
        pose_goal.position.y = 0.5
        pose_goal.position.z = 0.5
        pose_goal.orientation.w = 1
        move_group.set_pose_target(pose_goal)
        plan = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_target()
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)
    def go_to_joint_state(self):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -pi/4
        joint_goal[2] = 0
        joint_goal[3] = -pi/2
        joint_goal[4] = 0
        joint_goal[5] = pi/3
        joint_goal[6] = 0

        move_group.go(joint_goal, wait=True)
        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal,current_joints, 0.01)

def main():
    try:
        raw_input()
        start = MoveGroupInterface()
        # raw_input()
        # start.go_to_joint_state()
        raw_input()
        start.go_to_goal()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
  main()

