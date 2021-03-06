#!/usr/bin/env python

import sys
import time
import tf2_ros
import rospy
import moveit_commander
import geometry_msgs.msg
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped
import signal

from autolab_core import YamlConfig
from visualization import Visualizer2D as vis
from gqcnn.msg import GQCNNGrasp, BoundingBox
from gqcnn.srv import GQCNNGraspPlanner
from perception import RgbdDetectorFactory, RgbdSensorFactory
from autolab_core import RigidTransform
import joint_state_filereader


# Poses to boxes + Up state pose
# UP_POSE = geometry_msgs.msg.Pose(
#     position=Point(-0.449776958114, 0.146191358556, 1.02599018264),
#     orientation=Quaternion(-0.0225368166985, 0.975621319646, -0.217918914924, 0.00573890080621))

# UP_POSE = geometry_msgs.msg.Pose(
#     position=Point(-0.355011377996, 0.302698289576, 0.355856198301),
#     orientation=Quaternion(-0.628876449134, 0.318947102362, 0.638921332961, 0.307516971736))

# NEAR_BOX_POSE = geometry_msgs.msg.Pose(
#     # position=Point(-0.690099627989, 0.81849128041, 0.358930709262),
#     position=Point(-0.45, 0.60, 0.36),
#     orientation=Quaternion(0.493303531759, 0.542261814246, -0.514968722431, 0.44430953769))

# NEAR_DROP_POSE = geometry_msgs.msg.Pose(
#     position=Point(-1.04289410735, 0.322157005282, 0.220911818489),
#     orientation=Quaternion(0.22781686266, 0.689438716887, -0.313478313812, 0.611968201392))

# DROP_POSE = geometry_msgs.msg.Pose(
#     position=Point(-1.06653140818, 0.305945123927, 0.123294518075),
#     orientation=Quaternion(0.21940752316, 0.685697802181, -0.305246393463, 0.62330049105))

KEYBOARD_PAUSE = False


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

        filepath = rospy.get_param('~goal_joint_states')
        goal_joint_states = joint_state_filereader.read(filepath)
        self.goal_names = [
            "home", "near_box", "near_drop", "drop" 
        ]
        for name in self.goal_names:
            assert name in goal_joint_states, \
                "Joint state name '{}' is not found".format(name)

        # Declare suctionpad topics
        self.pub_to = rospy.Publisher('toArduino', String, queue_size=100)
        self.pub_from = rospy.Publisher('fromArduino', String, queue_size=100)

        # Vision config
        config = YamlConfig(rospy.get_param('~config'))

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

        # Setup vision sensor
        print("============ Setup vision sensor")
        # create rgbd sensor
        rospy.loginfo('Creating RGBD Sensor')
        sensor_cfg = config['sensor_cfg']
        sensor_type = sensor_cfg['type']
        self.sensor = RgbdSensorFactory.sensor(sensor_type, sensor_cfg)
        self.sensor.start()
        rospy.loginfo('Sensor Running')

        rospy.loginfo('Loading T_camera_world')
        tf_camera_world = RigidTransform.load(rospy.get_param('~world_camera_tf'))
        rospy.loginfo('tf_camera_world: {}'.format(tf_camera_world))

        # Setup client for grasp pose service
        rospy.loginfo('Setup client for grasp pose service')
        rospy.wait_for_service('grasping_policy')
        self.plan_grasp_client = rospy.ServiceProxy('grasping_policy', GQCNNGraspPlanner)

        self.transform_broadcaster = tf2_ros.TransformBroadcaster()

        # Misc variables
        self.robot = robot
        self.scene = scene
        self.group = group
        self.group_names = group_names
        self.config = config
        self.tf_camera_world = tf_camera_world
        self.goal_joint_states = goal_joint_states

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

    def go_to_state(self, joint_goal):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        group = self.group
        assert joint_goal is not None and len(joint_goal) == 6, \
            "Problem with goal joint state {}".format(joint_goal)
        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        group.stop()

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def plan_to_pose(self, pose_goal):
        group = self.group
        group.allow_replanning(True)
        group.set_planning_time(10)
        plan = group.plan(pose_goal)
        group.allow_replanning(False)
        group.set_planning_time(5)
        return plan

    def execute_plan(self, plan_msg, pose_goal):
        group = self.group
        if not group.execute(plan_msg, wait=True):
            return False

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
        self.pub_to.publish("SUCKER:ON")
        self.pub_to.publish("SUCKER:ON")

    def turn_off_suction_pad(self):
        self.pub_to.publish("SUCKER:OFF")
        self.pub_to.publish("SUCKER:OFF")
        self.pub_to.publish("SUCKER:OFF")

    def get_state(self, name):
        return self.goal_joint_states[name]

    def compute_object_pose(self):

        camera_intrinsics = self.sensor.ir_intrinsics
        # get the images from the sensor
        color_image, depth_image, _ = self.sensor.frames()
        # inpaint to remove holes
        inpainted_color_image = color_image.inpaint(rescale_factor=self.config['inpaint_rescale_factor'])
        inpainted_depth_image = depth_image.inpaint(rescale_factor=self.config['inpaint_rescale_factor'])

        detector_cfg = self.config['detector']
        detector_cfg['image_width'] = inpainted_depth_image.width // 3
        detector_cfg['image_height'] = inpainted_depth_image.height // 3
        detector = RgbdDetectorFactory.detector('point_cloud_box')
        detection = detector.detect(inpainted_color_image, inpainted_depth_image,
                                    detector_cfg,
                                    camera_intrinsics,
                                    self.tf_camera_world,
                                    vis_foreground=False,
                                    vis_segmentation=False)[0]

        if self.config['vis']['vis_detector_output']:
            vis.figure()
            vis.subplot(1, 2, 1)
            vis.imshow(detection.color_thumbnail)
            vis.subplot(1, 2, 2)
            vis.imshow(detection.depth_thumbnail)
            vis.show()

        boundingBox = BoundingBox()
        boundingBox.minY = detection.bounding_box.min_pt[0]
        boundingBox.minX = detection.bounding_box.min_pt[1]
        boundingBox.maxY = detection.bounding_box.max_pt[0]
        boundingBox.maxX = detection.bounding_box.max_pt[1]

        try:
            start_time = time.time()
            planned_grasp_data = self.plan_grasp_client(inpainted_color_image.rosmsg,
                                                        inpainted_depth_image.rosmsg,
                                                        camera_intrinsics.rosmsg,
                                                        boundingBox,
                                                        None)
            grasp_plan_time = time.time() - start_time
            rospy.loginfo("Planning time: {}".format(grasp_plan_time))
            rospy.loginfo("Camera CS planned_grasp_data:\n{}".format(planned_grasp_data.grasp.pose))

            if planned_grasp_data.grasp.q_value < 0.20:
                rospy.loginfo("Q value is too small : {}".format(planned_grasp_data.grasp.q_value))
                return None

            grasp_camera_pose = planned_grasp_data.grasp

            rospy.loginfo('Processing Grasp')
            self.broadcast_pose_as_transform(self.tf_camera_world.from_frame, "object_link", grasp_camera_pose.pose)

            # create Stamped ROS Transform
            camera_world_transform = TransformStamped()
            camera_world_transform.header.stamp = rospy.Time.now()
            camera_world_transform.header.frame_id = self.tf_camera_world.from_frame
            camera_world_transform.child_frame_id = self.tf_camera_world.to_frame

            camera_world_transform.transform.translation.x = self.tf_camera_world.translation[0]
            camera_world_transform.transform.translation.y = self.tf_camera_world.translation[1]
            camera_world_transform.transform.translation.z = self.tf_camera_world.translation[2]

            q = self.tf_camera_world.quaternion
            camera_world_transform.transform.rotation.x = q[1]
            camera_world_transform.transform.rotation.y = q[2]
            camera_world_transform.transform.rotation.z = q[3]
            camera_world_transform.transform.rotation.w = q[0]

            grasp_world_pose = tf2_geometry_msgs.do_transform_pose(grasp_camera_pose, camera_world_transform)

            # if grasp_world_pose.pose.position.z < 0.05:
            #     rospy.logwarn("Predicted pose Z value is less than 5 cm -> bad pose")
            #     return None

            # Hack z position
            # grasp_world_pose.pose.position.z += 0.0075
            # Hack orientation to 90 degree vertical pose
            grasp_world_pose.pose.orientation.x = -0.718339806303
            grasp_world_pose.pose.orientation.y = 0.00601026421019
            grasp_world_pose.pose.orientation.z = 0.695637686512
            grasp_world_pose.pose.orientation.w = 0.00632522789696

            rospy.loginfo("World CS planned_grasp_data:\n{}".format(grasp_world_pose))
            return grasp_world_pose.pose

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: \n %s" % e)
            return None

    def broadcast_pose_as_transform(self, frame_id, child_frame_id, pose):

        tf = TransformStamped()
        tf.header.stamp = rospy.Time.now()
        tf.header.frame_id = frame_id
        tf.child_frame_id = child_frame_id

        tf.transform.translation.x = pose.position.x
        tf.transform.translation.y = pose.position.y
        tf.transform.translation.z = pose.position.z

        tf.transform.rotation.x = pose.orientation.x
        tf.transform.rotation.y = pose.orientation.y
        tf.transform.rotation.z = pose.orientation.z
        tf.transform.rotation.w = pose.orientation.w

        self.transform_broadcaster.sendTransform(tf)


def main():


    try:
        program = PickAndDropProgram()

        # setup safe termination
        def handler(signum, frame):
            rospy.loginfo('caught CTRL+C, exiting...')
            if program.sensor is not None:
                program.sensor.stop()

            program.turn_off_suction_pad()
            program.go_to_state(program.get_state('home'))
            exit(0)

        signal.signal(signal.SIGINT, handler)


        print("============ Press `Enter` to initialize pose")
        if KEYBOARD_PAUSE:
            c = raw_input()
            if c == 'q':
                return
        else:
            time.sleep(0.33)

        program.go_to_state(program.get_state('home'))

        vision_fail_counter = 5
        while not rospy.is_shutdown():

            print("============ Press `Enter` to get object pose ...")
            if KEYBOARD_PAUSE:
                c = raw_input()
                if c == 'q':
                    break
            else:
                time.sleep(0.33)

            object_pose = program.compute_object_pose()
            if object_pose is None:
                vision_fail_counter -= 1
                if vision_fail_counter == 0:
                    break
                time.sleep(5)
                continue
            else:
                vision_fail_counter = 5

            print("============ Press `Enter` to go to near box pose ...")
            if KEYBOARD_PAUSE:
                c = raw_input()
                if c == 'q':
                    break
            else:
                time.sleep(0.33)
            if not program.go_to_state(program.get_state('near_box')):
                break

            print("============ Press `Enter` to turn on suction pad ...")

            if KEYBOARD_PAUSE:
                c = raw_input()
                if c == 'q':
                    break
            else:
                time.sleep(0.33)

            # turn on suction pad
            program.turn_on_suction_pad()

            print("============ Press `Enter` to plan to pick object pose ...")
            if KEYBOARD_PAUSE:
                c = raw_input()
                if c == 'q':
                    break
            else:
                time.sleep(0.33)

            plan_msg = program.plan_to_pose(object_pose)
            print("Plan nb of points", len(plan_msg.joint_trajectory.points))
            if len(plan_msg.joint_trajectory.points) == 0:
                program.turn_off_suction_pad()
                program.go_to_state(program.get_state('home'))
                continue

            print("============ Press `Enter` to execute to pick object pose ...")

            if KEYBOARD_PAUSE:
                c = raw_input()
                if c == 'q':
                    break
            else:
                time.sleep(0.33)

            if not program.execute_plan(plan_msg, object_pose):
                break

            print("============ Press `Enter` to go to near box pose ...")
            if KEYBOARD_PAUSE:
                c = raw_input()
                if c == 'q':
                    break
            else:
                time.sleep(0.33)

            if not program.go_to_state(program.get_state('near_box')):
                break

            print("============ Press `Enter` to go to near drop pose ...")
            if KEYBOARD_PAUSE:
                c = raw_input()
                if c == 'q':
                    break
            else:
                time.sleep(0.33)

            if not program.go_to_state(program.get_state('near_drop')):
                break

            print("============ Press `Enter` to go to drop pose ...")
            if KEYBOARD_PAUSE:
                c = raw_input()
                if c == 'q':
                    break
            else:
                time.sleep(0.33)

            if not program.go_to_state(program.get_state('drop')):
                break

            print("============ Press `Enter` to turn off suction pad ...")
            if KEYBOARD_PAUSE:
                c = raw_input()
                if c == 'q':
                    break
            else:
                time.sleep(0.33)
            # turn off suction pad
            program.turn_off_suction_pad()

            print("============ Press `Enter` to go to near drop pose ...")
            if KEYBOARD_PAUSE:
                c = raw_input()
                if c == 'q':
                    break
            else:
                time.sleep(0.33)
            if not program.go_to_state(program.get_state('near_drop')):
                break

            print("============ Press `Enter` to continue or `q` to quit ...")
            if KEYBOARD_PAUSE:
                c = raw_input()
                if c == 'q':
                    break
            else:
                time.sleep(0.33)

        program.turn_off_suction_pad()
        program.go_to_state(program.get_state('home'))
        print("============ Program complete!")

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()




