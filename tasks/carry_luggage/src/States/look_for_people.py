import smach
import rospy

from lasr_vision_msgs.srv import YoloDetectionRequest
from sensor_msgs.msg import Image, PointCloud2

class LookForPeople(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                             output_keys=['coords', 'pcl'])
        self.robot = default

    def execute(self, userdata):
        while True:
            if self.detection(userdata):
                return "succeeded"
            else:
                if self.robot.last_person_pose is not None:
                    return self.recovery(userdata)
            
    def recovery(self, userdata):
        self.robot.base_controller.sync_to_pose(self.robot.last_person_pose)

        result = self.recovery_lookout(self.robot.controllers.head_controller.look_right, userdata)
        if result == "succeeded":
            return result
        
        return self.recovery_lookout(self.robot.controllers.head_controller.look_left, userdata)
    
    def recovery_lookout(self, look_at, userdata):
        look_at()
        
        result = "failed"
        if self.detection(userdata):
            result = "succeeded"
            
        self.robot.controllers.head_controller.look_straight()
        return result

    def detection(self, userdata):
        msg = rospy.wait_for_message('/xtion/rgb/image_raw', Image)
        pcl = rospy.wait_for_message('/xtion/depth_registered/points', PointCloud2)

        request = YoloDetectionRequest()
        request.image_raw = msg                # sensor_msgs/Image
        request.dataset = "yolov8n-seg.pt"     # YOLOv8 model, auto-downloads
        request.confidence = 0.7               # minimum confidence to include in results
        request.nms = 0.4                      # non maximal supression

        # send request
        response = self.robot.detect_service(request)
        for detection in response.detected_objects:
            if detection.name == "person":
                self.robot.voice.speak("I see a person")

                # cords of person in image
                userdata.coords = detection.xyseg
                userdata.pcl = pcl
                return True
        
        return False