#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import logging
import signal
import time

from autolab_core import RigidTransform
from autolab_core import YamlConfig
from visualization import Visualizer2D as vis
import perception as perception
from perception import RgbdDetectorFactory, RgbdSensorFactory

from gqcnn.msg import GQCNNGrasp, BoundingBox
from sensor_msgs.msg import Image, CameraInfo
from gqcnn.srv import GQCNNGraspPlanner

import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped


def run_experiment():
    """ Run the experiment """
    print("run_experiment")

    rospy.loginfo('Wait for the service...')
    # wait for Grasp Planning Service and create Service Proxy
    rospy.wait_for_service('grasping_policy')
    plan_grasp = rospy.ServiceProxy('grasping_policy', GQCNNGraspPlanner)

    # create ROS CVBridge
    # cv_bridge = CvBridge()

    # get camera intrinsics
    # camera_intrinsics = sensor.color_intrinsics
    camera_intrinsics = sensor.ir_intrinsics
    rospy.loginfo('camera_intrinsics: {}'.format(camera_intrinsics.rosmsg))

    rospy.loginfo('Beginning experiment')

    # get the images from the sensor
    color_image, depth_image, _ = sensor.frames()

    # inpaint to remove holes
    inpainted_color_image = color_image.inpaint(rescale_factor=config['inpaint_rescale_factor'])
    inpainted_depth_image = depth_image.inpaint(rescale_factor=config['inpaint_rescale_factor'])

    print("inpainted_color_image: shape", inpainted_depth_image.shape)

    detector_cfg['image_width'] = inpainted_depth_image.width // 3
    detector_cfg['image_height'] = inpainted_depth_image.height // 3
    detector = RgbdDetectorFactory.detector('point_cloud_box')
    rospy.loginfo("Detect bbox")
    detection = detector.detect(
        inpainted_color_image, inpainted_depth_image,
        detector_cfg, camera_intrinsics,
        T_camera_world, vis_foreground=False, vis_segmentation=False)[0]

    if config['vis']['vis_detector_output']:
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
        rospy.loginfo("Start planning")
        start_time = time.time()
        planned_grasp_data = plan_grasp(inpainted_color_image.rosmsg,
                                        inpainted_depth_image.rosmsg,
                                        camera_intrinsics.rosmsg,
                                        boundingBox,
                                        None)
        grasp_plan_time = time.time() - start_time

        rospy.loginfo("Planning time: {}".format(grasp_plan_time))
        rospy.loginfo("planned_grasp_data:\n{}".format(planned_grasp_data.grasp.pose))

        grasp_camera_pose = planned_grasp_data.grasp
        # create Stamped ROS Transform
        camera_world_transform = TransformStamped()
        camera_world_transform.header.stamp = rospy.Time.now()
        camera_world_transform.header.frame_id = T_camera_world.from_frame
        camera_world_transform.child_frame_id = T_camera_world.to_frame

        camera_world_transform.transform.translation.x = T_camera_world.translation[0]
        camera_world_transform.transform.translation.y = T_camera_world.translation[1]
        camera_world_transform.transform.translation.z = T_camera_world.translation[2]

        q = T_camera_world.quaternion
        camera_world_transform.transform.rotation.x = q[1]
        camera_world_transform.transform.rotation.y = q[2]
        camera_world_transform.transform.rotation.z = q[3]
        camera_world_transform.transform.rotation.w = q[0]

        grasp_world_pose = tf2_geometry_msgs.do_transform_pose(grasp_camera_pose, camera_world_transform)
        rospy.loginfo("World CS planned_grasp_data:\n{}".format(grasp_world_pose))

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: \n %s" % e)


if __name__ == '__main__':
    
    # initialize the ROS node
    rospy.init_node('Kinect2_Client_Node')
    config = YamlConfig(rospy.get_param('~config'))

    # rospy.loginfo('Loading Gripper')
    # gripper = RobotGripper.load('yumi_metal_spline')

    rospy.loginfo('Loading T_camera_world')
    T_camera_world = RigidTransform.load(rospy.get_param('~world_camera_tf'))
    print("T_camera_world: ", T_camera_world)
    detector_cfg = config['detector']

    # create rgbd sensor
    rospy.loginfo('Creating RGBD Sensor')
    sensor_cfg = config['sensor_cfg']
    sensor_type = sensor_cfg['type']
    sensor = RgbdSensorFactory.sensor(sensor_type, sensor_cfg)
    sensor.start()
    rospy.loginfo('Sensor Running')

    # setup safe termination
    def handler(signum, frame):
        logging.info('caught CTRL+C, exiting...')
        if sensor is not None:
            sensor.stop()
        exit(0)
    signal.signal(signal.SIGINT, handler)

    counter = 1
    while counter > 0 and not rospy.is_shutdown():
        run_experiment()
        counter -= 0

