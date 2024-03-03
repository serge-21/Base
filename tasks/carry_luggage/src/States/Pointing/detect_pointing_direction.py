import smach

class DetectPointingDirection(smach.State):
    def __init__(self, default):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.robot = default

    def execute(self, userdata):
        resp = self.robot.pointing_detection_service()
        if resp.direction == "Err":
            return "failed"
        return "succeeded"
