import smach
import rospy

from lasr_vision_msgs.srv import YoloDetectionRequest
from sensor_msgs.msg import Image

class LookForPeople(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['succeeded', 'look_for_people'],
                             output_keys=['coords'])
        self.default = default

    def execute(self, userdata):
        msg = rospy.wait_for_message('/xtion/rgb/image_raw', Image)
        
        request = YoloDetectionRequest()
        request.image_raw = msg                # sensor_msgs/Image
        request.dataset = "yolov8n-seg.pt"     # YOLOv8 model, auto-downloads
        request.confidence = 0.7               # minimum confidence to include in results
        request.nms = 0.0                      # non maximal supression

        # send request
        response = self.default.detect_service(request)
        
        for detection in response.detected_objects:
            if detection.name == "person":
                self.default.voice.speak("I see a person")

                # cords of person in image
                userdata.coords = detection.xyseg
                return "succeeded"
            
        return "look_for_people"