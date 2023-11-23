import smach
import rospy

from lasr_vision_msgs.srv import YoloDetectionRequest
from sensor_msgs.msg import Image

class LookForPeople(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['succeeded'],
                             output_keys=['coords'])
        self.default = default
        self.motion = [
            self.default.controllers.head_controller.look_right,
            self.default.controllers.head_controller.look_straight,
            self.default.controllers.head_controller.look_left,
            self.default.controllers.head_controller.look_straight
        ]
        self.current_motion = 0

    def execute(self, userdata):
        # if we don't see a person, look around
        while True:
            self.motion[self.current_motion % len(self.motion)]()
            self.current_motion += 1

            if self.detection(userdata):
                return "succeeded"

    def detection(self, userdata):
        request = YoloDetectionRequest()
        msg = rospy.wait_for_message('/xtion/rgb/image_raw', Image)

        request.image_raw = msg                # sensor_msgs/Image
        request.dataset = "yolov8n-seg.pt"     # YOLOv8 model, auto-downloads
        request.confidence = 0.7               # minimum confidence to include in results
        request.nms = 0.4                      # non maximal supression

        # send request
        response = self.default.detect_service(request)
        print(response)
        for detection in response.detected_objects:
            if detection.name == "person":
                self.default.voice.speak("I see a person")

                # cords of person in image
                userdata.coords = detection.xyseg
                return True
        
        return False