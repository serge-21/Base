#!/usr/bin/env python3

import rospy
import numpy as np

from geometry_msgs.msg import Pose
from sensor_msgs import point_cloud2
from lasr_vision_msgs.srv import YoloDetection, YoloDetectionRequest
from sensor_msgs.msg import Image, PointCloud2

class Main:
    def __init__(self):
        self.detect_service = rospy.ServiceProxy('/yolov8/detect', YoloDetection)
        # wait for service to be available
        self.detect_service.wait_for_service()

        self.depth = None
        self.cam_subs = rospy.Subscriber("/xtion/rgb/image_raw", Image, self.request_test)
        self.depth_subs = rospy.Subscriber("/xtion/depth_registered/points", PointCloud2, self.do_nothing)

    def do_nothing(self, msg):
        self.depth = msg

    def estimate_depth(self, cords_of_person):
       # Check if depth information is available
        if self.depth is not None:
            try:
                # Convert PointCloud2 message to a numpy array
                depth_data = point_cloud2.read_points(self.depth, field_names=("x", "y", "z"), skip_nans=True)
                depth_array = np.array(list(depth_data))

                # Extract depth values for the coordinates of the person
                person_depth_values = []
                for cord in cords_of_person:
                    x, y = int(cord[0]), int(cord[1])
                    person_depth_values.append(depth_array[y, x, 2])  # Depth is the third element in the array

                # Calculate the average depth
                average_depth = np.mean(person_depth_values)

                rospy.loginfo("Estimated depth of the person: {}".format(average_depth))
                return average_depth

            except Exception as e:
                rospy.logerr("Error in estimating depth: {}".format(str(e)))

        else:
            rospy.logwarn("Depth information not available.")
            return None

    def estimate_pose(self, average_depth):
        return Pose()

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
                # depth = self.estimate_depth(cords_of_person)
                # pose = self.estimate_pose(depth)
                # self.base_controller.sync_to_pose(pose)

if __name__ == '__main__':
    rospy.init_node('go_to_person')
    yolo_ros = Main()
    rospy.spin()