#! /usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs.msg


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface', anonymous=True)
robot = moveit_commander.RobotCommander()

move_group = robot.get_group("arm")
print(move_group.get_end_effector_link())
move_group.set_end_effector_link("gripper_base")
print(move_group.get_end_effector_link())

arm_group = moveit_commander.MoveGroupCommander("arm")
hand_group = moveit_commander.MoveGroupCommander("hand")

hand_group.set_named_target("open")
plan2 = hand_group.go()

arm_group.set_named_target("mid1")
plan3 = arm_group.go()

arm_group.set_named_target("mid2")
plan4 = arm_group.go()

arm_group.set_named_target("mid3")
plan5 = arm_group.go()

hand_group.set_named_target("close")
plan6 = hand_group.go()

arm_group.set_named_target("mid4")
plan77 = arm_group.go()

rospy.sleep(10)
moveit_commander.roscpp_shutdown()