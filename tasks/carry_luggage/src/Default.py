#!/usr/bin/env python3

import rospy
from lasr_vision_msgs.srv import YoloDetection

from tf_module.srv import TfTransform, TfTransformRequest
from tiago_controllers.controllers import Controllers
from lasr_voice.voice import Voice
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import String

class Default:
    def __init__(self):
        self.detect_service = rospy.ServiceProxy('/yolov8/detect', YoloDetection)
        self.tf_service = rospy.ServiceProxy('tf_transform', TfTransform)
        self.controllers = Controllers()
        self.base_controller = self.controllers.base_controller

        self.initial_pose = self.controllers.base_controller.get_current_pose()
        self.voice = Voice()
        rospy.set_param('/is_simulation', False)
        
        self.last_person_pose = None

    def translate_coord_to_map(self, cords, header):
        x, y, z = cords

        point = PointStamped()
        point.point = Point(x, y, z)
        point.header = header
        
        tf_req = TfTransformRequest()
        tf_req.target_frame = String("map")
        tf_req.point = point

        response = self.tf_service(tf_req)

        return response.target_point.point.x, response.target_point.point.y, 0.0