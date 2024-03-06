import smach
from carry_luggage.srv import PointingServiceRequest

class DetectPointingDirection(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['Left', 'Right', 'failed'])
        self.robot = default

    def execute(self, userdata):
        req = PointingServiceRequest()
        resp = self.robot.pointing_detection_service(req)
        
        if resp.direction == "Err" or resp.direction == "Front":
            return "failed"
        
        return resp.direction
