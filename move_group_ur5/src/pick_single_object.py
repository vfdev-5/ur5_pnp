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


# Joint states
B_DST = [-1.68102840504, -2.7516480452, -0.411491741073, -1.516517344, 1.56106977053, 0.326503828761]
P0 = [0.130486115813, -1.60768586794, 0.083080291748, -1.61089498201, -1.56815272966, 0.0104277739301]
B4 = [0.83226776123, -0.936690632497, 1.20602989197, -1.83532506624, -1.57098180452, 0.0103326495737]
B5 = [1.14265239239, -1.2028482596, 1.57726335526, -1.97930938402, -1.57124501864, 0.0103086810559]
B6 = [1.56082582474, -1.25413304964, 1.64277172089, -2.06095391909, -1.57122117678, 0.010296696797]
B7 = [0.278196930885, -1.43176108996, 1.93985509872, -1.83550483385, -1.57092172304, 0.0103086810559]
B1 = [-0.224480454122,-0.931432072316, 1.20032930374, -1.88720542589, -1.5680568854, 0.0107273794711]
B2 = [0.106198959053, -0.886115376149, 1.18990945816, -1.83540898958, -1.57093364397, 0.0103086810559]
B3 = [0.40519040823, -0.736042324697, 0.876862049103, -1.83528882662, -1.57096940676, 0.0103086810559]


# Poses to boxes + Up state pose
UP_POSE = geometry_msgs.msg.Pose(
    position=Point(-0.40145000397, 0.15504653497, 1.0260589979),
    orientation=Quaternion(-8.43829805525e-10, -1.3519668215e-05, -0.99999999853, 5.2511481916e-05))

# PICK_OBJ_POSE = geometry_msgs.msg.Pose(
#     position=Point(-0.837426274603, 0.313374427721, 0.157699732871),
#     orientation=Quaternion(-0.971556638458, 0.225508936341, 0.0712216973955, 0.0122836362442))


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


class MoveGroupTutorial(object):
    """MoveGroupTutorial"""
    def __init__(self):
        super(MoveGroupTutorial, self).__init__()

        # First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_tutorial_ur5', anonymous=True)

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

    def go_to_up_state(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        group = self.group

        joint_goal = group.get_current_joint_values()
        print(type(joint_goal), joint_goal)

        # joint goal is a list of 6 elements : (x,y,z,r,p,y) can be composed of pose_msg
        joint_goal[0] = 0
        joint_goal[1] = -pi * 0.5
        joint_goal[2] = 0
        joint_goal[3] = -pi * 0.5
        joint_goal[4] = 0
        joint_goal[5] = 0

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        group.stop()

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_joints = self.group.get_current_joint_values()
        res = all_close(joint_goal, current_joints, 0.01)

        joint_goal = group.get_current_joint_values()
        current_pose = group.get_current_pose().pose
        print("Current pose: geometry_msgs.msg.Pose(position=({}, {}, {}),\norientation=({}, {}, {}, {}))"
              .format(current_pose.position.x, current_pose.position.y, current_pose.position.z,
                      current_pose.orientation.x, current_pose.orientation.y,
                      current_pose.orientation.z, current_pose.orientation.w,))
        print("Current joint state: ", joint_goal)
        return res

    def go_to_state(self, joint_state):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        group = self.group

        # joint goal is a list of 6 elements : (x,y,z,r,p,y) can be composed of pose_msg
        joint_goal = joint_state

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        group.stop()

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_joints = self.group.get_current_joint_values()
        res = all_close(joint_goal, current_joints, 0.01)

        joint_goal = group.get_current_joint_values()
        current_pose = group.get_current_pose().pose
        print("Current pose: geometry_msgs.msg.Pose(position=({}, {}, {}),\norientation=({}, {}, {}, {}))"
              .format(current_pose.position.x, current_pose.position.y, current_pose.position.z,
                      current_pose.orientation.x, current_pose.orientation.y,
                      current_pose.orientation.z, current_pose.orientation.w,))
        print("Current joint state: ", joint_goal)
        return res

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

        current_pose = group.get_current_pose().pose
        print("New current pose: ", current_pose)

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self, scale=1):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        group = self.group

        current_pose = group.get_current_pose().pose
        print("Current pose: ", current_pose)

        # Cartesian Paths
        # You can plan a Cartesian path directly by specifying a list of waypoints
        # for the end-effector to go through:
        waypoints = []

        wpose = group.get_current_pose().pose
        wpose.position.x = 0.2
        wpose.position.y = 0.01
        wpose.position.z = 0.2
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y += 0.1
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= 0.2
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
        (plan, fraction) = group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # eef_step
            0.0)         # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        # Displaying a Trajectory
        # You can ask RViz to visualize a plan (aka trajectory) for you. But the
        # group.plan() method does this automatically so this is not that useful
        # here (it just displays the same trajectory again):
        #
        # A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        # We populate the trajectory_start with our current robot state to copy over
        # any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory);

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        group = self.group

        # Executing a Plan
        # Use execute if you would like the robot to follow
        # the plan that has already been computed:
        group.execute(plan, wait=True)

        # **Note:** The robot's current joint state must be within some tolerance of the
        # first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        # If the Python node dies before publishing a collision object update message, the message
        # could get lost and the box will not appear. To ensure that the updates are
        # made, we wait until we see the changes reflected in the
        # ``get_known_object_names()`` and ``get_known_object_names()`` lists.
        # For the purpose of this tutorial, we call this function after adding,
        # removing, attaching or detaching an object in the planning scene. We then wait
        # until the updates have been made or ``timeout`` seconds have passed
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False

    def add_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        # First, we will create a box in the planning scene at the location of the left finger:
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "panda_leftfinger"
        box_pose.pose.orientation.w = 1.0
        box_name = "box"
        scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))

        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        self.box_name=box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        # Next, we will attach the box to the Panda wrist. Manipulating objects requires the
        # robot be able to touch them without the planning scene reporting the contact as a
        # collision. By adding link names to the ``touch_links`` array, we are telling the
        # planning scene to ignore collisions between those links and the box. For the Panda
        # robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
        # you should change this value to the name of your end effector group name.
        grasping_group = 'hand'
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    def detach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene
        eef_link = self.eef_link

        # We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name=box_name)

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

    def remove_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        # We can remove the box from the world.
        scene.remove_world_object(box_name)

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)


def main():

    try:
        print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ..."
        raw_input()
        tutorial = MoveGroupTutorial()

        print "============ Press `Enter` to execute a movement using a joint state goal ..."
        raw_input()
        tutorial.go_to_up_state()

        print "============ Press `Enter` to execute a movement using a joint state goal ..."
        raw_input()
        tutorial.go_to_pose(BOX_1_POSE)

        new_pose = copy.deepcopy(BOX_1_POSE)
        euler = euler_from_quaternion(
            (new_pose.orientation.x, new_pose.orientation.y, new_pose.orientation.z, new_pose.orientation.w)
        )
        # euler = (R, P, Y)
        euler = list(euler)
        euler[1] += 0.5
        new_pose.orientation = Quaternion(*quaternion_from_euler(euler[0], euler[1], euler[2]))

        print "============ Press `Enter` to execute a movement using a joint state goal ..."
        raw_input()
        tutorial.go_to_pose(new_pose)


        print "============ Press `Enter` to execute a movement using a joint state goal ..."
        raw_input()
        tutorial.go_to_pose(BOX_2_POSE)

        print "============ Press `Enter` to execute a movement using a joint state goal ..."
        raw_input()
        tutorial.go_to_pose(BOX_3_POSE)

        print "============ Press `Enter` to execute a movement using a joint state goal ..."
        raw_input()
        tutorial.go_to_pose(BOX_4_POSE)

        print "============ Press `Enter` to execute a movement using a joint state goal ..."
        raw_input()
        tutorial.go_to_pose(BOX_5_POSE)

        print "============ Press `Enter` to execute a movement using a joint state goal ..."
        raw_input()
        tutorial.go_to_pose(BOX_6_POSE)

        print "============ Press `Enter` to execute a movement using a joint state goal ..."
        raw_input()
        tutorial.go_to_pose(BOX_7_POSE)

        print "============ Press `Enter` to execute a movement using a joint state goal ..."
        raw_input()
        tutorial.go_to_pose(BOX_DST_POSE)

        print "============ Press `Enter` to execute a movement using a joint state goal ..."
        raw_input()
        tutorial.go_to_pose(UP_POSE)



        # print "============ Press `Enter` to plan and display a Cartesian path ..."
        # raw_input()
        # cartesian_plan, fraction = tutorial.plan_cartesian_path()
        #
        # print "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
        # raw_input()
        # tutorial.display_trajectory(cartesian_plan)
        #
        # print "============ Press `Enter` to execute a saved path ..."
        # raw_input()
        # tutorial.execute_plan(cartesian_plan)

        # print "============ Press `Enter` to add a box to the planning scene ..."
        # raw_input()
        # tutorial.add_box()

        # print "============ Press `Enter` to attach a Box to the Panda robot ..."
        # raw_input()
        # tutorial.attach_box()

        # print "============ Press `Enter` to plan and execute a path with an attached collision object ..."
        # raw_input()
        # cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
        # tutorial.execute_plan(cartesian_plan)

        # print "============ Press `Enter` to detach the box from the Panda robot ..."
        # raw_input()
        # tutorial.detach_box()

        # print "============ Press `Enter` to remove the box from the planning scene ..."
        # raw_input()
        # tutorial.remove_box()

        print "============ Python tutorial demo complete!"
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()




