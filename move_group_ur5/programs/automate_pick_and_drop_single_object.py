#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Point, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

# Poses to boxes + Up state pose
UP_POSE = geometry_msgs.msg.Pose(
    position=Point(-0.44995, 0.155801542599, 1.02605862083),
    orientation=Quaternion(0.707106556984, -0.000563088017201, -0.707106556987, 0.000563088010273))

NEAR_BOX_POSE = geometry_msgs.msg.Pose(
    position=Point(-0.675139039609, 0.867279417402, 0.279982326596),
    orientation=Quaternion(-0.868894712561, 0.48837298469, 0.0705417807789, 0.0392130523262))

DROP_POSE = geometry_msgs.msg.Pose(
    position=Point(-1.09533408607, 0.336971544257, 0.205691443581),
    orientation=Quaternion(-0.184521466167, -0.9803796679, 0.0115032200886, 0.0683755162301))

PICK_OBJ_POSE = geometry_msgs.msg.Pose(
    position=Point(-0.68115685593, 0.876995807511, 0.10939952704),
    orientation=Quaternion(-0.871625146062, 0.490012399741, 0.0111440867713, 0.005767337592))


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


class PickAndDropProgram(object):
    
    def __init__(self):
        super(PickAndDropProgram, self).__init__()

        # First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('pick_and_drop_program', anonymous=True)

        # Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
        # the robot:
        robot = moveit_commander.RobotCommander()

        # Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
        # to the world surrounding the robot:
        scene = moveit_commander.PlanningSceneInterface()

        # Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        # to one group of joints.  In this case the group is the joints in the UR5
        # arm so we set ``group_name = manipulator``. If you are using a different robot,
        # you should change this value to the name of your robot arm planning group.
        group_name = "manipulator" # See .srdf file to get available group names
        group = moveit_commander.MoveGroupCommander(group_name)

        # We create a `DisplayTrajectory`_ publisher which is used later to publish
        # trajectories for RViz to visualize:
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        # We can get the name of the reference frame for this robot:
        planning_frame = group.get_planning_frame()
        print "============ Reference frame: %s" % planning_frame

        # We can also print the name of the end-effector link for this group:
        eef_link = group.get_end_effector_link()
        print "============ End effector: %s" % eef_link

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print "============ Robot Groups:", robot.get_group_names()

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print "============ Printing robot state"
        print robot.get_current_state()
        print ""

        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_pose(self, pose_goal):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        group = self.group

        # We can plan a motion for this group to a desired pose for the end-effector:
        group.set_pose_target(pose_goal)

        # Now, we call the planner to compute the plan and execute it.
        plan = group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        group.clear_pose_targets()

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)


def main():

    try:
        program = PickAndDropProgram()

        print("============ Press `Enter` to initialize pose")
        raw_input()
        program.go_to_pose(UP_POSE)

        is_running = True
        while is_running:
            print("============ Press `Enter` to go to near box pose ...")
            raw_input()
            program.go_to_pose(NEAR_BOX_POSE)

            print("============ Press `Enter` to turn on suction pad ...")
            raw_input()
            # turn on suction pad

            print("============ Press `Enter` to go to pick object pose ...")
            raw_input()
            program.go_to_pose(PICK_OBJ_POSE)

            print("============ Press `Enter` to go to near box pose ...")
            raw_input()
            program.go_to_pose(NEAR_BOX_POSE)

            print("============ Press `Enter` to go to drop pose ...")
            raw_input()
            program.go_to_pose(DROP_POSE)

            print("============ Press `Enter` to turn off suction pad ...")
            raw_input()
            # turn off suction pad

            print("============ Press `Enter` to continue or `q` to quit ...")
            c = raw_input()
            if c == 'q':
                is_running = False

        program.go_to_pose(UP_POSE)
        print "============ Program complete!"

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()




