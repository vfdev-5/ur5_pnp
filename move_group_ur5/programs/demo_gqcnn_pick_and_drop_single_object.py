#!/usr/bin/env python

import sys
import time
import numpy as np
import rospy
import moveit_commander
import geometry_msgs.msg
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped

from autolab_core import YamlConfig
from visualization import Visualizer2D as vis
from gqcnn.msg import GQCNNGrasp, BoundingBox
from gqcnn.srv import GQCNNGraspPlanner
from perception import RgbdDetectorFactory, RgbdSensorFactory
from autolab_core import RigidTransform


# Poses to boxes + Up state pose
UP_POSE = geometry_msgs.msg.Pose(
    position=Point(-0.449776958114, 0.146191358556, 1.02599018264),
    orientation=Quaternion(0.0225368166985, -0.808053659539, -0.588442707886, 0.0166299348762))

NEAR_BOX_POSE = geometry_msgs.msg.Pose(
    position=Point(-0.690099627989, 0.81849128041, 0.358930709262),
    orientation=Quaternion(-0.0262376625131, 0.752089932142, 0.0988363827275, 0.651032927931))

NEAR_DROP_POSE = geometry_msgs.msg.Pose(
    position=Point(-1.07128657834, 0.320749025386, 0.20939021189),
    orientation=Quaternion(-0.322710358702, 0.646787770911, 0.332593909725, 0.60572674945))

DROP_POSE = geometry_msgs.msg.Pose(
    position=Point(-1.10543750452, 0.329165925663, 0.111933751523),
    orientation=Quaternion(-0.297015016999, 0.670153720059, 0.359937433728, 0.577166453434))


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

        # Misc variables
        self.robot = robot
        self.scene = scene
        self.group = group
        self.group_names = group_names
        self.config = config
        self.tf_camera_world = tf_camera_world

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

    def compute_object_pose(self):

        camera_intrinsics = self.sensor.ir_intrinsics
        # get the images from the sensor
        color_image, depth_image, _ = self.sensor.frames()
        # inpaint to remove holes
        inpainted_color_image = color_image.inpaint(rescale_factor=self.config['inpaint_rescale_factor'])
        inpainted_depth_image = depth_image.inpaint(rescale_factor=self.config['inpaint_rescale_factor'])

        detector_cfg = self.config['detector']
        detector_cfg['image_width'] = inpainted_depth_image.width // 10
        detector_cfg['image_height'] = inpainted_depth_image.height // 10
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

            grasp_camera_pose = planned_grasp_data.grasp
            rospy.loginfo('Processing Grasp')

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
            rospy.loginfo("World CS planned_grasp_data:\n{}".format(grasp_world_pose))
            return grasp_world_pose.pose

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: \n %s" % e)
            return None


def main():

    try:
        program = PickAndDropProgram()

        print("============ Press `Enter` to initialize pose")
        raw_input()
        program.go_to_pose(UP_POSE)

        vision_fail_counter = 5
        while True:

            print("============ Press `Enter` to get object pose ...")
            raw_input()
            object_pose = program.compute_object_pose()
            if object_pose is None:
                vision_fail_counter -= 1
                if vision_fail_counter == 0:
                    break
                continue
            else:
                vision_fail_counter = 5

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
            if not program.go_to_pose(object_pose):
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




