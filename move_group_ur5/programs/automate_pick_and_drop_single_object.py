#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

# Poses to boxes + Up state pose
UP_POSE = geometry_msgs.msg.Pose(
    position=Point(-0.449776958114, 0.146191358556, 1.02599018264),
    orientation=Quaternion(-0.0225368166985, 0.975621319646, -0.217918914924, 0.00573890080621))

NEAR_BOX_POSE = geometry_msgs.msg.Pose(
    position=Point(-0.690099627989, 0.81849128041, 0.358930709262),
    orientation=Quaternion(0.493303531759, 0.542261814246, -0.514968722431, 0.44430953769))

NEAR_DROP_POSE = geometry_msgs.msg.Pose(
    position=Point(-1.04289410735, 0.322157005282, 0.220911818489),
    orientation=Quaternion(0.22781686266, 0.689438716887, -0.313478313812, 0.611968201392))

DROP_POSE = geometry_msgs.msg.Pose(
    position=Point(-1.06653140818, 0.305945123927, 0.123294518075),
    orientation=Quaternion(0.21940752316, 0.685697802181, -0.305246393463, 0.62330049105))

PICK_OBJ_POSE = geometry_msgs.msg.Pose(
    position=Point(-0.632689452737, 0.872960752869, 0.140723604517),
    orientation=Quaternion(0.491085000724, 0.502232066142, -0.526102378448, 0.479389988625))


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
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

        # Declare suctionpad topics
        self.pub_to = rospy.Publisher('toArduino', String, queue_size=100)
        self.pub_from = rospy.Publisher('fromArduino', String, queue_size=100)

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
        # display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
        #                                                moveit_msgs.msg.DisplayTrajectory,
        #                                                queue_size=20)

        # We can get the name of the reference frame for this robot:
        planning_frame = group.get_planning_frame()
        print("============ Reference frame: %s" % planning_frame)

        # We can also print(the name of the end-effector link for this group:
        eef_link = group.get_end_effector_link()
        print("============ End effector: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Robot Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")

        # Misc variables
        self.robot = robot
        self.scene = scene
        self.group = group
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

    def turn_on_suction_pad(self):
        self.pub_to.publish("SUCKER:ON")

    def turn_off_suction_pad(self):
        self.pub_to.publish("SUCKER:OFF")


def main():

    try:
        program = PickAndDropProgram()

        print("============ Press `Enter` to initialize pose")
        raw_input()
        program.go_to_pose(UP_POSE)

        while True:
            print("============ Press `Enter` to go to near box pose ...")
            c = raw_input()
            if c == 'q':
                break
            if not program.go_to_pose(NEAR_BOX_POSE):
                break

            print("============ Press `Enter` to turn on suction pad ...")
            c = raw_input()
            if c == 'q':
                break
            # turn on suction pad
            program.turn_on_suction_pad()

            print("============ Press `Enter` to go to pick object pose ...")
            c = raw_input()
            if c == 'q':
                break
            if not program.go_to_pose(PICK_OBJ_POSE):
                break

            print("============ Press `Enter` to go to near box pose ...")
            c = raw_input()
            if c == 'q':
                break
            if not program.go_to_pose(NEAR_BOX_POSE):
                break

            print("============ Press `Enter` to go to near drop pose ...")
            c = raw_input()
            if c == 'q':
                break
            if not program.go_to_pose(NEAR_DROP_POSE):
                break

            print("============ Press `Enter` to go to drop pose ...")
            c = raw_input()
            if c == 'q':
                break
            if not program.go_to_pose(DROP_POSE):
                break

            print("============ Press `Enter` to turn off suction pad ...")
            c = raw_input()
            if c == 'q':
                break
            # turn off suction pad
            program.turn_off_suction_pad()

            print("============ Press `Enter` to go to near drop pose ...")
            c = raw_input()
            if c == 'q':
                break
            if not program.go_to_pose(NEAR_DROP_POSE):
                break

            print("============ Press `Enter` to continue or `q` to quit ...")
            c = raw_input()
            if c == 'q':
                break

        program.turn_off_suction_pad()
        program.go_to_pose(UP_POSE)
        print("============ Program complete!")

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()




