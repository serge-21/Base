#!/usr/bin/env python3

import rospy
from lasr_vision_msgs.srv import YoloDetection

from tf_module.srv import TfTransform
from tiago_controllers.controllers import Controllers
from lasr_voice.voice import Voice

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
