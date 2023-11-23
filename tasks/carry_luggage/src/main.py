#!/usr/bin/env python3

import rospy
import numpy as np

from geometry_msgs.msg import Pose, PointStamped, Point
from sensor_msgs import point_cloud2
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2
from lasr_vision_msgs.srv import YoloDetectionRequest, YoloDetection
from tf_module.srv import TfTransformRequest, TfTransform

from tiago_controllers.controllers import Controllers

class Main:
    def __init__(self):
        self.detect_service = rospy.ServiceProxy('/yolov8/detect', YoloDetection)
        self.detect_service.wait_for_service()
        self.tf_service = rospy.ServiceProxy('tf_transform', TfTransform)
        self.tf_service.wait_for_service()

        self.baseController = Controllers().base_controller

        self.depth = None
        self.initial_pose = self.baseController.get_current_pose()
        self.cam_subs = rospy.Subscriber("/xtion/rgb/image_raw", Image, self.request_test)
        self.depth_subs = rospy.Subscriber("/xtion/depth_registered/points", PointCloud2, self.do_nothing)

    def do_nothing(self, msg):
        self.depth = msg

    def estimate_person_coords(self, cords_of_person):
       # Check if depth information is available
        if self.depth is not None:
            try:
                # Convert PointCloud2 message to a numpy array
                depth_data = point_cloud2.read_points(self.depth, field_names=("x", "y", "z"), skip_nans=True)
                depth_array = np.array(list(depth_data))

                # Extract depth values for the coordinates of the person
                person_depth_values_x = []
                person_depth_values_y = []
                person_depth_values_z = []
                for cord in cords_of_person:
                    x, y, z = depth_array[cord]
                    person_depth_values_x.append(x)
                    person_depth_values_y.append(y)
                    person_depth_values_z.append(z)

                # Calculate the average depth
                average_depth_x = np.mean(person_depth_values_x)
                average_depth_y = np.mean(person_depth_values_y)
                average_depth_z = np.mean(person_depth_values_z)

                rospy.loginfo("Estimated depth of the person: {}".format((average_depth_x, average_depth_y, average_depth_z)))
                return (average_depth_x, average_depth_y, average_depth_z)

            except Exception as e:
                rospy.logerr("Error in estimating depth: {}".format(str(e)))

        else:
            rospy.logwarn("Depth information not available.")
            return None

    def estimate_pose(self, person_cords):
        x, y, _ = person_cords
        centroid = PointStamped()

        centroid.point = Point(x, y, 0)
        centroid.header = self.depth.header
        
        tf_req = TfTransformRequest()
        tf_req.target_frame = String("map")
        tf_req.point = centroid
        
        response = self.tf_service(tf_req)
        
        target_pose = Pose()
        target_pose.position.x = response.target_point.point.x
        target_pose.position.y = response.target_point.point.y
        target_pose.position.z = 0.0
        target_pose.orientation.w = 1.0

        return target_pose


    def request_test(self, msg):
        # create request
        request = YoloDetectionRequest()
        request.image_raw = msg                # sensor_msgs/Image
        request.dataset = "yolov8n-seg.pt"     # YOLOv8 model, auto-downloads
        request.confidence = 0.7               # minimum confidence to include in results
        request.nms = 0.0                      # non maximal supression

        # send request
        response = self.detect_service(request)
        
        for detection in response.detected_objects:
            if detection.name == "person":
                rospy.loginfo("Found person")

                # cords of person in image
                cords_of_person = detection.xyseg
                person_cords = self.estimate_person_coords(cords_of_person)
                pose = self.estimate_pose(person_cords)

                rospy.loginfo("initial pose: {}".format(self.initial_pose))
                rospy.loginfo("pose of person: {}".format(pose))
                
                self.baseController.sync_to_pose(pose)

if __name__ == '__main__':
    rospy.init_node('go_to_person')
    yolo_ros = Main()
    rospy.spin()